// getftbias - get a bunch of readings from the force transducer at rest
// calculate an average to use as ft_raw
//
// InMotion2 robot system software for RTLinux

// Copyright 2003-2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

// EXIT_FAILURE
#include <stdlib.h>

// mbuffs
#include "/usr/src/linux-2.4.18-rtl3.1/rtlinux-3.1/include/mbuff.h"

// // primitive typedefs
#include "ruser.h"

// robot decls
#include "robdecls.h"

#define ITERS 200

// pointers to shared buffer objects
int
main() {
    Ob *ob;
    Robot *rob;
    Daq *daq;

    s32 c[6];		// ft channel
    f64 volts[6];	// raw volts
    s32 i, j;		// loop indices

    printf("this is obsolete, just bias the ft with ft_dobias.");
    printf("getftbias will run for about 8 seconds...\n\n");

    system ("./go");	// load the lkm

    // attach to shm
    if ((ob = (Ob *) mbuff_attach("ob", (int)(sizeof(Ob)))) == NULL) {
	fprintf(stderr, "couldn't mbuff_alloc shared memory structure ob\n"
	"(the robot kernel module is probably not running)\n");
	// _exit now, not exit.
            _exit (EXIT_FAILURE);
    }
    rob = (Robot *) mbuff_attach("rob", (int)(sizeof(Robot)));
    daq = (Daq *) mbuff_attach("daq", (int)(sizeof(Daq)));

    // un-pause.
    ob->paused = 0;

    // wait for un-pause to take effect
    // usleeps are not exact, but are minimum.
    usleep(50*1000L);	// sleep 1/20 sec, make sure it's unpaused.

    // find out which ft.channel is which
    for (i=0; i<6; i++) {
	volts[i] = 0;
	c[i] = rob->ft.channel[i];
    }

    // read the ft voltages ITERS times,
    // sleeping in between, to make sure you don't reread the same sample
    for (j=0; j<ITERS; j++) {
	for (i=0; i<6; i++) {
	    volts[i] += rob->ft.raw[i];
	}
	usleep(20*1000L);	// sleep 1/50 sec
    }

    // pause
    ob->paused = 1;

    usleep(50*1000L);	// sleep 1/20 sec, make sure it's paused.

    // release mbuffs
    mbuff_detach("rob", rob);
    mbuff_detach("daq", daq);
    mbuff_detach("ob", ob);

    // unload lkm
    system ("./stop");

    // print average bias values
    for (i=0; i<6; i++) {
	printf("s ft_bias %d %f\n", i, -volts[i]/ITERS);
    }
    // make it easier to copy/paste
    printf("\n");

    // and exit
    exit(0);
}
