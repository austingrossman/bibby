#define _GNU_SOURCE
#include "web_logs.h"

#include <dirent.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

// Columns pulled out of a log, in the order the JSON reports them. Looked up
// by name in the CSV header, so a log written by an older or newer schema
// still plots whatever columns it does have.
static const char *const kCols[] = {
  "t_monotonic_s", "temp_raw_c", "temp_filt_c", "setpoint_c",
  "p_demand_w", "p_delivered_w", "duty1", "duty2",
  "pid_ff_w", "pid_p_w", "pid_i_w", "pid_d_w",
  "m_est_l", "manual", "grain_in", "rtd_fault", "watchdog",
};
#define NCOL ((int)(sizeof(kCols) / sizeof(kCols[0])))

const char *web_logs_root(void) {
  static char root[512];
  if (!root[0]) {
    const char *home = getenv("HOME");
    if (!home || !*home) home = ".";
    snprintf(root, sizeof(root), "%s/bibby/logs", home);
  }
  return root;
}

// ── Listing ───────────────────────────────────────────────────────────────────

typedef struct {
  char   rel[256];
  long long size;
  long long mtime;
} LogEntry;

typedef struct {
  LogEntry *v;
  int       n, cap;
} LogList;

static void list_push(LogList *l, const char *rel, long long size, long long mtime) {
  if (l->n == l->cap) {
    int cap = l->cap ? l->cap * 2 : 128;
    LogEntry *v = realloc(l->v, (size_t)cap * sizeof(*v));
    if (!v) return;
    l->v = v;
    l->cap = cap;
  }
  snprintf(l->v[l->n].rel, sizeof(l->v[l->n].rel), "%s", rel);
  l->v[l->n].size  = size;
  l->v[l->n].mtime = mtime;
  l->n++;
}

// Recursive walk of YYYY/MM/DD/*.csv. Depth-limited so a stray symlink loop
// cannot spin the server.
static void scan_dir(LogList *l, const char *abs, const char *rel, int depth) {
  if (depth > 4 || l->n > 5000) return;
  DIR *d = opendir(abs);
  if (!d) return;
  struct dirent *e;
  while ((e = readdir(d))) {
    if (e->d_name[0] == '.') continue;
    char sub_abs[1024], sub_rel[256];
    // A path too long to hold is a path bibby did not write; skip it rather
    // than build a truncated one that would not open anyway.
    if (snprintf(sub_abs, sizeof(sub_abs), "%s/%s", abs, e->d_name) >=
            (int)sizeof(sub_abs) ||
        snprintf(sub_rel, sizeof(sub_rel), "%s%s%s", rel, rel[0] ? "/" : "",
                 e->d_name) >= (int)sizeof(sub_rel))
      continue;
    struct stat sb;
    if (stat(sub_abs, &sb) != 0) continue;
    if (S_ISDIR(sb.st_mode)) {
      scan_dir(l, sub_abs, sub_rel, depth + 1);
    } else if (S_ISREG(sb.st_mode)) {
      size_t n = strlen(e->d_name);
      if (n > 4 && !strcmp(e->d_name + n - 4, ".csv"))
        list_push(l, sub_rel, (long long)sb.st_size, (long long)sb.st_mtime);
    }
  }
  closedir(d);
}

static int by_mtime_desc(const void *a, const void *b) {
  const LogEntry *x = a, *y = b;
  if (x->mtime != y->mtime) return x->mtime < y->mtime ? 1 : -1;
  return strcmp(y->rel, x->rel);
}

void web_logs_list_json(Buf *out, const char *active_path) {
  LogList l = {0};
  scan_dir(&l, web_logs_root(), "", 0);
  if (l.n > 1) qsort(l.v, (size_t)l.n, sizeof(*l.v), by_mtime_desc);

  // The live run is identified by its absolute path; match on the tail so the
  // comparison survives the logger and the server spelling the root the same.
  const char *active_tail = NULL;
  if (active_path && *active_path) {
    size_t rlen = strlen(web_logs_root());
    if (!strncmp(active_path, web_logs_root(), rlen) && active_path[rlen] == '/')
      active_tail = active_path + rlen + 1;
  }

  buf_puts(out, "[");
  for (int i = 0; i < l.n; i++) {
    char when[32];
    time_t t = (time_t)l.v[i].mtime;
    struct tm lt;
    localtime_r(&t, &lt);
    strftime(when, sizeof(when), "%Y-%m-%d %H:%M:%S", &lt);
    buf_printf(out,
               "%s{\"path\":\"%s\",\"bytes\":%lld,\"mtime\":%lld,"
               "\"when\":\"%s\",\"active\":%s}",
               i ? "," : "", l.v[i].rel, l.v[i].size, l.v[i].mtime, when,
               (active_tail && !strcmp(active_tail, l.v[i].rel)) ? "true" : "false");
  }
  buf_puts(out, "]");
  free(l.v);
}

// ── Path validation ───────────────────────────────────────────────────────────

bool web_logs_resolve(const char *rel_path, char *out, size_t out_size) {
  if (!rel_path || !*rel_path || rel_path[0] == '/') return false;
  size_t n = strlen(rel_path);
  if (n < 5 || strcmp(rel_path + n - 4, ".csv") != 0) return false;
  if (strstr(rel_path, "..")) return false;

  char joined[1024];
  snprintf(joined, sizeof(joined), "%s/%s", web_logs_root(), rel_path);

  // realpath() is the authority: it collapses links and any traversal the
  // textual checks above missed, and the result must still sit under the root.
  char real_file[PATH_MAX], real_root[PATH_MAX];
  if (!realpath(joined, real_file) || !realpath(web_logs_root(), real_root))
    return false;
  size_t rlen = strlen(real_root);
  if (strncmp(real_file, real_root, rlen) != 0 || real_file[rlen] != '/')
    return false;

  struct stat sb;
  if (stat(real_file, &sb) != 0 || !S_ISREG(sb.st_mode)) return false;
  snprintf(out, out_size, "%s", real_file);
  return true;
}

// ── Series decimation ─────────────────────────────────────────────────────────
// One streaming pass, bounded memory: rows accumulate into fixed-size buckets;
// when the bucket array fills, adjacent buckets are merged pairwise and the
// rows-per-bucket doubles. The result is a whole-file average at whatever
// resolution fits, so a 400 k-row tuning log costs one read and no seeking.

typedef struct {
  double sum[NCOL];
  int    n;
} Bucket;

static void bucket_merge_pairs(Bucket *b, int *count) {
  int out = 0;
  for (int i = 0; i + 1 < *count; i += 2, out++) {
    for (int c = 0; c < NCOL; c++) b[out].sum[c] = b[i].sum[c] + b[i + 1].sum[c];
    b[out].n = b[i].n + b[i + 1].n;
  }
  if (*count & 1) b[out++] = b[*count - 1];
  *count = out;
}

bool web_logs_series_json(Buf *out, const char *rel_path, int max_points) {
  char path[PATH_MAX];
  if (!web_logs_resolve(rel_path, path, sizeof(path))) return false;
  FILE *f = fopen(path, "r");
  if (!f) return false;

  if (max_points < 50)   max_points = 50;
  if (max_points > 5000) max_points = 5000;

  char  *line = NULL;
  size_t line_cap = 0;
  ssize_t got = getline(&line, &line_cap, f);
  if (got <= 0) { free(line); fclose(f); return false; }

  // Map each wanted column to its position in this file's header.
  int idx[NCOL];
  for (int c = 0; c < NCOL; c++) idx[c] = -1;
  int ncols = 0;
  for (char *tok = strtok(line, ",\r\n"); tok; tok = strtok(NULL, ",\r\n"), ncols++)
    for (int c = 0; c < NCOL; c++)
      if (idx[c] < 0 && !strcmp(tok, kCols[c])) idx[c] = ncols;

  const int cap = max_points;   // merging halves it, so the result is <= this
  Bucket *b = calloc((size_t)cap, sizeof(*b));
  if (!b) { free(line); fclose(f); return false; }

  int      count = 0;          // buckets in use
  long     rows_per_bucket = 1;
  long long rows = 0;
  double   t_first = 0, t_last = 0;
  bool     have_t = false;
  char     wall_start[40] = "";

  while ((got = getline(&line, &line_cap, f)) > 0) {
    if (line[0] == '\n' || line[0] == '\r' || line[0] == '\0') continue;

    // Split in place; a field is only converted if some series wants it.
    double val[64];
    int    nf = 0;
    char  *p = line;
    char  *field = p;
    for (; nf < 64; p++) {
      if (*p != ',' && *p != '\n' && *p != '\r' && *p != '\0') continue;
      char term = *p;
      *p = '\0';
      if (nf == 0 && !wall_start[0])
        snprintf(wall_start, sizeof(wall_start), "%s", field);
      val[nf++] = atof(field);
      field = p + 1;
      if (term != ',') break;
    }

    if (count == 0 || b[count - 1].n >= rows_per_bucket) {
      if (count == cap) {
        bucket_merge_pairs(b, &count);
        rows_per_bucket *= 2;
        memset(b + count, 0, (size_t)(cap - count) * sizeof(*b));
      }
      count++;
    }
    Bucket *cur = &b[count - 1];
    for (int c = 0; c < NCOL; c++) {
      int i = idx[c];
      if (i >= 0 && i < nf) cur->sum[c] += val[i];
    }
    cur->n++;

    if (idx[0] >= 0 && idx[0] < nf) {
      if (!have_t) { t_first = val[idx[0]]; have_t = true; }
      t_last = val[idx[0]];
    }
    rows++;
  }
  free(line);
  fclose(f);

  buf_printf(out,
             "{\"path\":\"%s\",\"rows\":%lld,\"points\":%d,"
             "\"duration_s\":%.1f,\"wall_start\":\"%s\",\"series\":{",
             rel_path, rows, count, have_t ? t_last - t_first : 0.0, wall_start);
  for (int c = 0; c < NCOL; c++) {
    buf_printf(out, "%s\"%s\":", c ? "," : "", kCols[c]);
    if (idx[c] < 0) { buf_puts(out, "null"); continue; }
    buf_puts(out, "[");
    for (int i = 0; i < count; i++) {
      double mean = b[i].n ? b[i].sum[c] / b[i].n : 0.0;
      // Column 0 is the time base: report it relative to the start of the run.
      if (c == 0) mean -= t_first;
      buf_printf(out, "%s%.4g", i ? "," : "", mean);
    }
    buf_puts(out, "]");
  }
  buf_puts(out, "}}");
  free(b);
  return !out->oom;
}
