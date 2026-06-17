#pragma once
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/file.h>
#include <unistd.h>

// Enforce a single running instance of a program using an advisory file lock.
//
// Call once at startup with a unique lock path (e.g. "/tmp/biab_heater.lock").
// On success it returns a file descriptor that must stay open for the entire
// life of the process: the lock is held as long as the fd is open, and the
// kernel releases it automatically when the process exits for any reason,
// including a crash or kill -9. No stale lock files to clean up.
//
// Returns the lock fd (>= 0) if this is the only instance, or -1 if another
// instance already holds the lock or the lock file could not be opened.
static inline int single_instance_lock(const char *path) {
  int fd = open(path, O_RDWR | O_CREAT, 0644);
  if (fd < 0) {
    perror("single_instance: open");
    return -1;
  }
  if (flock(fd, LOCK_EX | LOCK_NB) < 0) {
    if (errno != EWOULDBLOCK)
      perror("single_instance: flock");
    close(fd);
    return -1;
  }
  return fd;
}
