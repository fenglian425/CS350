#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>
#include <queue.h>

/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */
static struct lock *commonLock;
static struct cv *commonCV;
static struct cv *w_cv;
static struct cv *e_cv;
static struct cv *n_cv;
static struct cv *s_cv;
volatile int count;
volatile int westC;
volatile int eastC;
volatile int northC;
volatile int southC;
volatile Direction d;
static struct queue *q;
volatile bool westQ;
volatile bool eastQ;
volatile bool northQ;
volatile bool southQ;


/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{

  commonLock = lock_create("commonLock");
  if (commonLock == NULL) {
    panic("could not create common lock");
  }

  commonCV = cv_create("commonCV");
  if (commonCV == NULL) {
    panic("could not create common cv");
  }
  
  w_cv = cv_create("west");
  if (commonCV == NULL) {
    panic("could not create west cv");
  }

  e_cv = cv_create("east");
  if (commonCV == NULL) {
    panic("could not create east cv");
  }

  n_cv = cv_create("north");
  if (commonCV == NULL) {
    panic("could not create north cv");
  }

  s_cv = cv_create("south");
  if (commonCV == NULL) {
    panic("could not create south cv");
  }

  q = q_create(4);
  if (q == NULL) {
    panic("could not create a queue");
  }

  count = 0;
  westC = 0;
  eastC = 0;
  northC = 0;
  southC = 0;
  westQ = false;
  eastQ = false;
  northQ = false;
  southQ = false;
  d = north;
  return;
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{

  KASSERT(commonLock != NULL);
  KASSERT(commonCV != NULL);
  KASSERT(w_cv != NULL);
  KASSERT(e_cv != NULL);
  KASSERT(n_cv != NULL);
  KASSERT(s_cv != NULL);
  KASSERT(q != NULL);
  cv_destroy(commonCV);
  cv_destroy(w_cv);
  cv_destroy(e_cv);
  cv_destroy(n_cv);
  cv_destroy(s_cv);
  lock_destroy(commonLock);
  q_destroy(q);
  /* replace this default implementation with your own implementation */
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination) 
{

  lock_acquire(commonLock);
  if (count == 0) {
    count ++;
    d = origin;
  }
  else if (origin == west) {
    count ++;
    westC ++;
    if (origin != d) {
      if (westQ == false) {
        q_addtail(q, w_cv);
        westQ = true;
      }
      cv_wait(w_cv, commonLock);
    }
  }
  else if (origin == east) {
    count ++;
    eastC ++;
    if (origin != d) {
      if (eastQ == false) {
        q_addtail(q, e_cv);
        eastQ = true;
      }
      cv_wait(e_cv, commonLock);
    }
  }
  else if (origin == north) {
    count++;
    northC ++;
    if (origin != d) {
      if (northQ == false) {
        q_addtail(q, n_cv);
        northQ = true;
      }
      cv_wait(n_cv, commonLock);
    }
  }
  else {
    count ++;
    southC ++;
    if (origin != d) {
      if (southQ == false) {
        q_addtail(q, s_cv);
        southQ = true;
      }
      cv_wait(s_cv, commonLock);
    }
  }

  lock_release(commonLock);
  /* replace this default implementation with your own implementation */
  (void)origin;  /* avoid compiler complaint about unused parameter */
  (void)destination; /* avoid compiler complaint about unused parameter */
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{

  lock_acquire(commonLock);
  bool dequeue = false;
  count --;
  struct cv* temp_cv = NULL;
  if (!q_empty(q)) {
    temp_cv = q_peek(q);
  }
  if (origin == west) {
    westC --;
    if ((westC == 0) && (q_len(q) != 0)) {
      cv_broadcast(temp_cv, commonLock);
      q_remhead(q);
      dequeue = true;
    }
  }
  else if (origin == east) {
    eastC --;
    if ((eastC == 0) && (q_len(q) != 0)) {
      cv_broadcast(temp_cv, commonLock);
      q_remhead(q);
      dequeue = true;
    }
  }
  else if (origin == north) {
    northC --;
    if ((northC == 0) && (q_len(q) != 0)) {
      cv_broadcast(temp_cv, commonLock);
      q_remhead(q);
      dequeue = true;
    }
  }
  else {
    southC --;
    if ((southC == 0) && (q_len(q) != 0)) {
      cv_broadcast(temp_cv, commonLock);
      q_remhead(q);
      dequeue = true;
    }
  }
  if (dequeue == true) {
    if (temp_cv == w_cv) {
      d = west;
      westQ = false;
    }
    else if (temp_cv == e_cv) { 
      eastQ = false;
      d = east; 
    }
    else if (temp_cv == n_cv) { 
      northQ = false;
      d = north;
    }
    else {
      southQ = false;
      d = south;
    }
  }
  temp_cv = NULL;

  lock_release(commonLock);

  /* replace this default implementation with your own implementation */
  (void)origin;  /* avoid compiler complaint about unused parameter */
  (void)destination; /* avoid compiler complaint about unused parameter */
}
