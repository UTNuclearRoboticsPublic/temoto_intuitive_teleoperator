// Copyright (c) 2015-2016, The University of Texas at Austin
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
// 
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
// 
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <linux/input.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <glob.h>	// for counting files in a dir (needed for counting event files in /dev/input)

#include "ros/ros.h"
#include "std_msgs/String.h"

// temoto includes
#include "temoto_include.h"

/*	This code is originally from http://sowerbutts.com/powermate/
 *	Comments added by karl.kruusamae(at)utexas.edu
 *	Modifications to make it into a ROS publisher and other changes made by karl.kruusamae(at)utexas.edu
 * 
 *	If you get permission denied when starting this node. Use ' ls -l /dev/input/event* ' to learn which group can access the events.
 *	Then add your username to this group with ' sudo usermod -a -G group_name user_name '
 */

#define NUM_VALID_PREFIXES 2
#define BUFFER_SIZE 32

long long total_shift = 0;	///< Integrated shift of Powermate dial.
//signed char dir = 0;
signed char pressed = 0;	///< True when Powermate dial is pressed.

/** Array of valid prefixes for Griffin devices. */
static const char *valid_prefix[NUM_VALID_PREFIXES] = {
  "Griffin PowerMate",
  "Griffin SoundKnob"
};

/** Opens the input device and checks whether its meaningful name (ie, EVIOCGNAME in ioctl()) is Griffin PowerMate.
 *  @param dev file name for linux event.
 *  @param mode open file mode.
 *  @return file descriptor if all checks out, otherwise -1.
 */
int open_powermate(const char *dev, int mode) {
//  ROS_INFO("Opening dev %s", dev);
  int fd = open(dev, mode);	//file descriptor to the opened device
  int i;
  char name[255];		// meaningful, ie EVIOCGNAME name

  if(fd < 0){			// if failed to open dev
    fprintf(stderr, "Unable to open \"%s\": %s\n", dev, strerror(errno));
    return -1;
  }

  if(ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0){ 	// fetches the meaningful (ie. EVIOCGNAME) name
    fprintf(stderr, "\"%s\": EVIOCGNAME failed: %s\n", dev, strerror(errno));
    close(fd);
    return -1;						// returns -1 if unable to fetch the meaningful name
  }

  //checks now if the meaningful name matches one listed in valid_prefix
  // it's the correct device if the prefix matches what we expect it to be:
  for(i=0; i<NUM_VALID_PREFIXES; i++)
    if( !strncasecmp(name, valid_prefix[i], strlen(valid_prefix[i])) ) {
      ROS_INFO("Found %s device. Starting to read ...", valid_prefix[i]);
      return fd;					// if everything checks out, returns the file  tdescriptor
    } // end if
  close(fd);
  return -1;
} // end open_powermate

/** Goes through all the event files in /dev/input/ to locate powermate.
 *  @param mode open file mode.
 *  @return file descriptor if all checks out, otherwise -1.
 */
int find_powermate(int mode) {
  int i, r;

  // using glob() [see: http://linux.die.net/man/3/glob ] for getting event files in /dev/input/
  glob_t gl;
  int num_event_dev = 0;					// number of event files found in /dev/input/
  if(glob("/dev/input/event*", GLOB_NOSORT, NULL, &gl) == 0)	// counts for filenames that match the given pattern
    num_event_dev = gl.gl_pathc;				// get number of event files

  for(i=0; i<num_event_dev; i++){				// go through all found event files
    r = open_powermate(gl.gl_pathv[i], mode);			// try to open an event file
    if(r >= 0)							// if opened file is a powermate event return file descriptor
      return r;
  } // for
  globfree(&gl);						// free memory allocated for globe struct

  return -1;							// return error -1 otherwise
} // end find_powermate

/** Reads the event type and publishes relevant data.
 *  @param ev input event.
 *  @param pub ROS publisher.
 */
void process_event(struct input_event *ev, ros::Publisher& pub) {
#ifdef VERBOSE
  fprintf(stderr, "type=0x%04x, code=0x%04x, value=%d\n",
	  ev->type, ev->code, (int)ev->value);
#endif

  temoto::Dial msg;				// msg will be published by ROS publisher

// Event information about Griffin PowerMate from evtest
//
// Input device name: "Griffin PowerMate"
// Supported events:
//	Event type 0 (EV_SYN)
//	Event type 1 (EV_KEY)
//		Event code 256 (BTN_0)
//	Event type 2 (EV_REL)
//		Event code 7 (REL_DIAL)
//	Event type 4 (EV_MSC)
//		Event code 1 (MSC_PULSELED)
  
  switch(ev->type){				// switch to a case based on the event type
    case EV_SYN:				// no need to do anything
//      printf("SYN REPORT\n");
      break; 
    case EV_MSC:				// unused for this ROS publisher
      printf("The LED pulse settings were changed; code=0x%04x, value=0x%08x\n", ev->code, ev->value);
      break;
    case EV_REL:				// ROS publisher: getting rotational data
      if(ev->code != REL_DIAL)
	fprintf(stderr, "Warning: unexpected rotation event; ev->code = 0x%04x\n", ev->code);
      else {
	signed char dir = (signed char)ev->value;// reads direction value from dial
	total_shift += (long long)dir;		// integrates consecutive dir values to find total shift
	msg.direction = dir;			// putting msg together
	msg.integral = total_shift;
	msg.pressed = pressed;
	msg.push_ev_occured = false;
	pub.publish(msg);			// publish msg
	//printf("Button was rotated %d units; Shift from start is now %d units\n", (int)ev->value, total_shift);
      }
      break;
    case EV_KEY:				// ROS publisher: getting data about pressing and depressing the dial button
      if(ev->code != BTN_0)
	fprintf(stderr, "Warning: unexpected key event; ev->code = 0x%04x\n", ev->code);
      else {
	//printf("Button was %s\n", ev->value? "pressed":"released");
	pressed = (signed char)ev->value;	// reads EV_KEY value
	msg.direction = 0;			// putting msg together; in the case of KEY EVENT, direction is 0
	msg.integral = total_shift;
	msg.pressed = pressed;
	msg.push_ev_occured = true;
	pub.publish(msg);			// publish msg
      }
      break;
    default:					// default case
      fprintf(stderr, "Warning: unexpected event type; ev->type = 0x%04x\n", ev->type);
  } // end switch

  fflush(stdout);
} // end process_event

/** Method for initiating ROS publisher and reading the event data.
 *  @param fd file descriptor for powermate event file.
 */
void watch_powermate(int fd) {

  ros::NodeHandle nh("~");

  // Creates publisher that advertises Dial messages on rostopic /griffin_powermate/events
  ros::Publisher pub_powermate_events = nh.advertise<temoto::Dial>("events", 100);

  struct input_event ibuffer[BUFFER_SIZE];				// see: https://www.kernel.org/doc/Documentation/input/input.txt
  int r, events, i;

  while(ros::ok()){
//    ROS_INFO("ROS OK begin while");
    
    // read() reads a binary file [http://pubs.opengroup.org/onlinepubs/9699919799/functions/read.html] and returns the number of bytes read.
    // The program waits in read() until there's something to read; thus it always gets a new event but ROS cannot make a clean exit while in read(). It's not a big problem.
    r = read(fd, ibuffer, sizeof(struct input_event) * BUFFER_SIZE);
    if( r > 0 ) {
      events = r / sizeof(struct input_event);				// getting the number of events
      for(i=0; i<events; i++)						// going through all the read events
	process_event(&ibuffer[i], pub_powermate_events);			// call process_event() for every read event
    } else {
	fprintf(stderr, "read() failed: %s\n", strerror(errno));	// let user know that read() failed
	return;
    }
    
  } // end while
  
  return;
} // end watch_powermate

/** Main method. */
int main(int argc, char *argv[]) {
  
  ros::init(argc, argv, "griffin_powermate");
  ros::AsyncSpinner spinner(1);				// using async spinner
  spinner.start();					// starting async spinner
  
  int powermate = -1;					// by default, powermate is considered not opened

  if(argc == 1)
    powermate = find_powermate(O_RDONLY); 		// if no arguments, find powermate from existing input events
  else
    powermate = open_powermate(argv[1], O_RDONLY);	// open the given input event

  if(powermate < 0){					// if failed to open any powermate input event, print info and exit
    fprintf(stderr, "Unable to locate powermate\n");
    fprintf(stderr, "Try: %s [device]\n", argv[0]);
    return 1;
  }

  watch_powermate(powermate);				// if powermate is succesfully opened, read its input

  close(powermate);					// close powermate

  return 0;
} //end main
