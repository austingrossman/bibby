#define _GNU_SOURCE
#include "http.h"

#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

// ── Growable buffer ───────────────────────────────────────────────────────────

static bool buf_reserve(Buf *b, size_t extra) {
  if (b->oom) return false;
  if (b->len + extra + 1 <= b->cap) return true;
  size_t cap = b->cap ? b->cap : 4096;
  while (cap < b->len + extra + 1) cap *= 2;
  char *p = realloc(b->p, cap);
  if (!p) { b->oom = true; return false; }
  b->p = p;
  b->cap = cap;
  return true;
}

bool buf_append(Buf *b, const void *data, size_t len) {
  if (!buf_reserve(b, len)) return false;
  memcpy(b->p + b->len, data, len);
  b->len += len;
  b->p[b->len] = '\0';
  return true;
}

bool buf_puts(Buf *b, const char *s) { return buf_append(b, s, strlen(s)); }

bool buf_printf(Buf *b, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  char stack[512];
  int n = vsnprintf(stack, sizeof(stack), fmt, ap);
  va_end(ap);
  if (n < 0) return false;
  if ((size_t)n < sizeof(stack)) return buf_append(b, stack, (size_t)n);

  // Rare long value: format again straight into the buffer.
  if (!buf_reserve(b, (size_t)n)) return false;
  va_start(ap, fmt);
  vsnprintf(b->p + b->len, (size_t)n + 1, fmt, ap);
  va_end(ap);
  b->len += (size_t)n;
  return true;
}

void buf_free(Buf *b) {
  free(b->p);
  b->p = NULL;
  b->len = b->cap = 0;
  b->oom = false;
}

// ── Percent decoding ──────────────────────────────────────────────────────────

static int hexval(int c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Decode `src` (up to src_len bytes) into out. NUL bytes are dropped so a
// decoded string can never be shorter than it looks to later strcmp/strstr.
static void percent_decode(const char *src, size_t src_len, char *out,
                           size_t out_size) {
  size_t o = 0;
  for (size_t i = 0; i < src_len && o + 1 < out_size; i++) {
    int c = (unsigned char)src[i];
    if (c == '%' && i + 2 < src_len) {
      int hi = hexval((unsigned char)src[i + 1]);
      int lo = hexval((unsigned char)src[i + 2]);
      if (hi >= 0 && lo >= 0) { c = hi * 16 + lo; i += 2; }
    } else if (c == '+') {
      c = ' ';
    }
    if (c == 0) continue;
    out[o++] = (char)c;
  }
  out[o] = '\0';
}

bool http_query_get(const HttpReq *r, const char *key, char *out, size_t out_size) {
  size_t klen = strlen(key);
  const char *p = r->query;
  while (*p) {
    const char *amp = strchr(p, '&');
    const char *end = amp ? amp : p + strlen(p);
    const char *eq  = memchr(p, '=', (size_t)(end - p));
    if (eq && (size_t)(eq - p) == klen && !strncmp(p, key, klen)) {
      percent_decode(eq + 1, (size_t)(end - eq - 1), out, out_size);
      return true;
    }
    if (!amp) break;
    p = amp + 1;
  }
  return false;
}

// ── Basic auth ────────────────────────────────────────────────────────────────

static bool base64_decode(const char *in, char *out, size_t out_size) {
  static const char tbl[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  unsigned acc = 0;
  int bits = 0;
  size_t o = 0;
  for (const char *p = in; *p && *p != '='; p++) {
    if (isspace((unsigned char)*p)) continue;
    const char *q = memchr(tbl, *p, 64);
    if (!q) return false;
    acc = (acc << 6) | (unsigned)(q - tbl);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      if (o + 1 >= out_size) return false;
      out[o++] = (char)((acc >> bits) & 0xFF);
    }
  }
  out[o] = '\0';
  return true;
}

bool http_basic_auth(const char *authorization, char *user, size_t user_size,
                     char *pass, size_t pass_size) {
  user[0] = pass[0] = '\0';
  if (strncasecmp(authorization, "Basic ", 6) != 0) return false;
  const char *b64 = authorization + 6;
  while (*b64 == ' ') b64++;

  char decoded[512];
  if (!base64_decode(b64, decoded, sizeof(decoded))) return false;
  char *colon = strchr(decoded, ':');
  if (!colon) return false;
  *colon = '\0';
  snprintf(user, user_size, "%s", decoded);
  snprintf(pass, pass_size, "%s", colon + 1);
  return true;
}

bool http_secret_equal(const char *a, const char *b) {
  size_t la = strlen(a), lb = strlen(b);
  size_t n = la > lb ? la : lb;
  unsigned diff = (unsigned)(la ^ lb);
  for (size_t i = 0; i < n; i++)
    diff |= (unsigned)((i < la ? a[i] : 0) ^ (i < lb ? b[i] : 0));
  return diff == 0;
}

// ── Requests ──────────────────────────────────────────────────────────────────

void http_req_init(HttpReq *r, int fd) {
  memset(r, 0, sizeof(*r));
  r->fd = fd;
}

// Case-insensitive "Header-Name:" test; returns the value start or NULL.
static const char *header_value(const char *line, const char *name) {
  size_t n = strlen(name);
  if (strncasecmp(line, name, n) != 0 || line[n] != ':') return NULL;
  const char *v = line + n + 1;
  while (*v == ' ' || *v == '\t') v++;
  return v;
}

bool http_read_request(HttpReq *r) {
  r->method[0] = r->path[0] = r->query[0] = r->authorization[0] = '\0';
  r->content_length = 0;
  r->body_len = 0;
  r->keep_alive = true;

  size_t head_end = 0;
  for (;;) {
    char *e = memmem(r->buf, r->buf_len, "\r\n\r\n", 4);
    if (e) { head_end = (size_t)(e - r->buf) + 4; break; }
    if (r->buf_len >= HTTP_MAX_HEADER) return false;
    ssize_t n = recv(r->fd, r->buf + r->buf_len, HTTP_MAX_HEADER - r->buf_len, 0);
    if (n <= 0) return false;
    r->buf_len += (size_t)n;
  }

  char head[HTTP_MAX_HEADER + 1];
  memcpy(head, r->buf, head_end);
  head[head_end] = '\0';

  // Request line: METHOD SP TARGET SP VERSION
  char *line_end = strstr(head, "\r\n");
  if (!line_end) return false;
  *line_end = '\0';
  char *sp1 = strchr(head, ' ');
  if (!sp1) return false;
  *sp1 = '\0';
  char *target = sp1 + 1;
  char *sp2 = strchr(target, ' ');
  if (!sp2) return false;
  *sp2 = '\0';
  const char *version = sp2 + 1;

  if (strlen(head) >= sizeof(r->method)) return false;   // no such HTTP method
  memcpy(r->method, head, strlen(head) + 1);
  if (!strncmp(version, "HTTP/1.0", 8)) r->keep_alive = false;

  char *qmark = strchr(target, '?');
  if (qmark) {
    *qmark = '\0';
    snprintf(r->query, sizeof(r->query), "%s", qmark + 1);
  }
  percent_decode(target, strlen(target), r->path, sizeof(r->path));

  for (char *p = line_end + 2; p && *p;) {
    char *eol = strstr(p, "\r\n");
    if (!eol || eol == p) break;
    *eol = '\0';
    const char *v;
    if ((v = header_value(p, "Authorization")))
      snprintf(r->authorization, sizeof(r->authorization), "%s", v);
    else if ((v = header_value(p, "Content-Length")))
      r->content_length = strtol(v, NULL, 10);
    else if ((v = header_value(p, "Connection")))
      r->keep_alive = strcasecmp(v, "close") != 0;
    p = eol + 2;
  }

  if (r->content_length < 0 || r->content_length > HTTP_MAX_BODY) {
    http_respond_text(r, 413, "request body too large");
    return false;
  }

  size_t want = (size_t)r->content_length;
  size_t have = r->buf_len - head_end;
  size_t take = have < want ? have : want;
  memcpy(r->body, r->buf + head_end, take);
  r->body_len = take;

  size_t consumed = head_end + take;
  r->buf_len -= consumed;
  memmove(r->buf, r->buf + consumed, r->buf_len);

  while (r->body_len < want) {
    ssize_t n = recv(r->fd, r->body + r->body_len, want - r->body_len, 0);
    if (n <= 0) return false;
    r->body_len += (size_t)n;
  }
  r->body[r->body_len] = '\0';
  return true;
}

// ── Responses ─────────────────────────────────────────────────────────────────

bool http_send_all(int fd, const void *data, size_t len) {
  const char *p = data;
  while (len > 0) {
    ssize_t n = send(fd, p, len, MSG_NOSIGNAL);
    if (n <= 0) return false;
    p   += n;
    len -= (size_t)n;
  }
  return true;
}

static const char *status_text(int status) {
  switch (status) {
    case 200: return "OK";
    case 204: return "No Content";
    case 304: return "Not Modified";
    case 400: return "Bad Request";
    case 401: return "Unauthorized";
    case 403: return "Forbidden";
    case 404: return "Not Found";
    case 405: return "Method Not Allowed";
    case 413: return "Payload Too Large";
    case 500: return "Internal Server Error";
    case 503: return "Service Unavailable";
    default:  return "Error";
  }
}

void http_respond(HttpReq *r, int status, const char *content_type,
                  const void *body, size_t len, const char *extra_headers) {
  char head[1024];
  int n = snprintf(head, sizeof(head),
                   "HTTP/1.1 %d %s\r\n"
                   "Content-Type: %s\r\n"
                   "Content-Length: %zu\r\n"
                   "Cache-Control: no-store\r\n"
                   "Connection: %s\r\n"
                   "%s\r\n",
                   status, status_text(status), content_type, len,
                   r->keep_alive ? "keep-alive" : "close",
                   extra_headers ? extra_headers : "");
  if (n < 0 || (size_t)n >= sizeof(head)) { r->keep_alive = false; return; }
  if (!http_send_all(r->fd, head, (size_t)n)) { r->keep_alive = false; return; }
  if (len && strcmp(r->method, "HEAD") != 0 &&
      !http_send_all(r->fd, body, len))
    r->keep_alive = false;
}

void http_respond_text(HttpReq *r, int status, const char *text) {
  http_respond(r, status, "text/plain; charset=utf-8", text, strlen(text), NULL);
}

void http_respond_file(HttpReq *r, const char *fs_path, const char *content_type,
                       const char *download_name) {
  int fd = open(fs_path, O_RDONLY);
  struct stat sb;
  if (fd < 0 || fstat(fd, &sb) != 0 || !S_ISREG(sb.st_mode)) {
    if (fd >= 0) close(fd);
    http_respond_text(r, 404, "no such log file");
    return;
  }

  char extra[640];
  snprintf(extra, sizeof(extra),
           "Content-Disposition: attachment; filename=\"%s\"\r\n", download_name);

  char head[1024 + sizeof(extra)];
  int n = snprintf(head, sizeof(head),
                   "HTTP/1.1 200 OK\r\n"
                   "Content-Type: %s\r\n"
                   "Content-Length: %lld\r\n"
                   "Cache-Control: no-store\r\n"
                   "Connection: %s\r\n"
                   "%s\r\n",
                   content_type, (long long)sb.st_size,
                   r->keep_alive ? "keep-alive" : "close", extra);
  if (n < 0 || (size_t)n >= sizeof(head) ||
      !http_send_all(r->fd, head, (size_t)n)) {
    r->keep_alive = false;
    close(fd);
    return;
  }

  if (!strcmp(r->method, "HEAD")) { close(fd); return; }

  char chunk[65536];
  off_t sent = 0;
  while (sent < sb.st_size) {
    ssize_t got = read(fd, chunk, sizeof(chunk));
    if (got <= 0) break;
    // Never write past the advertised length if the live log grew mid-send.
    if (sent + got > sb.st_size) got = (ssize_t)(sb.st_size - sent);
    if (!http_send_all(r->fd, chunk, (size_t)got)) { r->keep_alive = false; break; }
    sent += got;
  }
  if (sent != sb.st_size) r->keep_alive = false;  // truncated: connection is unusable
  close(fd);
}
