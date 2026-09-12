#pragma once
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>

// Minimal HTTP/1.1 plumbing for the bibby web interface: request parsing,
// response writing, Basic-auth decoding and a growable output buffer. Nothing
// here knows about brewing; web.c does the routing.
//
// Scope on purpose: one request at a time per connection, Content-Length
// bodies only (no chunked uploads), fixed header/body ceilings. This serves a
// handful of LAN clients on the brew panel, not the open internet.

#define HTTP_MAX_HEADER 8192
#define HTTP_MAX_BODY   8192

typedef struct {
  int    fd;
  char   method[8];
  char   path[512];            // percent-decoded, query stripped
  char   query[512];           // raw query string (still percent-encoded)
  char   authorization[512];   // raw Authorization header value, "" if absent
  long   content_length;
  bool   keep_alive;
  char   body[HTTP_MAX_BODY + 1];
  size_t body_len;

  // Carry-over bytes between keep-alive requests; do not touch from callers.
  char   buf[HTTP_MAX_HEADER + 1];
  size_t buf_len;
} HttpReq;

// Zero a connection state for a freshly accepted socket.
void http_req_init(HttpReq *r, int fd);

// Read one request (headers + body). False on EOF, timeout, or a malformed or
// oversized request — the caller should close the connection either way.
bool http_read_request(HttpReq *r);

bool http_send_all(int fd, const void *data, size_t len);

// One complete response. `extra_headers`, when non-NULL, is inserted verbatim
// and must end with CRLF.
void http_respond(HttpReq *r, int status, const char *content_type,
                  const void *body, size_t len, const char *extra_headers);
void http_respond_text(HttpReq *r, int status, const char *text);

// Response whose body is streamed from an open file (log downloads).
void http_respond_file(HttpReq *r, const char *fs_path, const char *content_type,
                       const char *download_name);

// Look up a query parameter, percent-decoding the value. False if absent.
bool http_query_get(const HttpReq *r, const char *key, char *out, size_t out_size);

// Decode "Basic <base64>" into user/pass. False if the header is missing or
// not well-formed Basic credentials.
bool http_basic_auth(const char *authorization, char *user, size_t user_size,
                     char *pass, size_t pass_size);

// Length-independent comparison, so a wrong password cannot be narrowed down
// by timing the response.
bool http_secret_equal(const char *a, const char *b);

// ── Growable byte buffer (JSON assembly) ─────────────────────────────────────
typedef struct { char *p; size_t len, cap; bool oom; } Buf;

bool buf_append(Buf *b, const void *data, size_t len);
bool buf_puts(Buf *b, const char *s);
bool buf_printf(Buf *b, const char *fmt, ...);
void buf_free(Buf *b);
