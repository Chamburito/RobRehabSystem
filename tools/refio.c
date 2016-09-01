//
// InMotion2 robot system software for Realtime Linux

// Copyright 2009 Interactive Motion Technologies, Inc.
// Watertown, MA, USA
// http://www.interactive-motion.com
// All rights reserved

// refio
// read or write data between robot reference memory and a file

// EXIT_FAILURE
#include <stdlib.h>

// signals
#include <signal.h>
 
// primitive typedefs
#include "ruser.h"
#include "rtl_inc.h"
// robot decls
#include "robdecls.h"

// pointers to shared buffer objects
Ob *ob;
Refbuf *refbuf;

void do_atexit(int);

void
usage(void) {
    printf("usage: refio -r (or -w) filename\n");
}

void
init(void)
{
    int ob_shmid;
    int refbuf_shmid;

    // do_atexit calls mbuff_free.
    // leaving mbuffs allocated is a bad thing.
    (void) signal(1, (__sighandler_t) do_atexit);
    (void) signal(2, (__sighandler_t) do_atexit);
    (void) signal(15, (__sighandler_t) do_atexit);
    (void) atexit((void *) do_atexit);

    ob_shmid = shmget(OB_KEY, sizeof(Ob), 0666);
    if (ob_shmid == -1) {
      fprintf(stderr, "could not shmget() access to shared memory ob\n");
      exit(EXIT_FAILURE);
    }
    ob = (Ob *)shmat(ob_shmid, NULL, 0);
    if ((s32)ob == -1) {
      fprintf(stderr, "ob shmat() failed\n");
      exit(EXIT_FAILURE);
    }

    refbuf_shmid = shmget(REFBUF_KEY, sizeof(Refbuf), 0666);
    if (refbuf_shmid == -1) {
      fprintf(stderr, "could not shmget() access to shared memory refbuf\n");
      exit(EXIT_FAILURE);
    }
    refbuf = (Refbuf *)shmat(refbuf_shmid, NULL, 0);
    if ((s32)refbuf == -1) {
      fprintf(stderr, "refbuf shmat() failed\n");
      exit(EXIT_FAILURE);
    }
}

void do_atexit(int sig)
{
    // TODO: delete mbuff_detach("ob", ob);
    // TODO: delete mbuff_detach("rob", rob);
    // TODO: delete mbuff_detach("daq", daq);
    shmdt(ob);
    shmdt(refbuf);

    // don't call the atexit stuff atain!
    _exit(0);
}

char *filename;

void
do_read(void)
{
    FILE *ifd;
    s32 size;

    init();

    ifd = fopen (filename, "r");
    fseek(ifd, 0L, SEEK_END);
    size = ftell(ifd);
    ob->refterm = size / (5*sizeof(f64));
    fseek(ifd, 0L, SEEK_SET);
    fread(refbuf->refarr, size, 1, ifd);

    fclose(ifd);
}

void
do_write(void)
{
    FILE *ofd;
    f64 n;

    init();
    n = ob->refterm;

    ofd = fopen (filename, "a");
    fwrite(refbuf->refarr, (5*sizeof(f64)), n, ofd);

    fclose(ofd);
}

s32
main(int argc, char **argv)
{
    if (argc != 3) {
	usage();
	exit(1);
    }

    filename = argv[2];

    if (!strcmp(argv[1], "-r")) {
	do_read();
	// printf("reading\n");
    } else if  (!strcmp(argv[1], "-w")) {
	do_write();
	// printf("writing\n");
    } else {
	usage();
	exit(1);
    }
    return 0;
}
