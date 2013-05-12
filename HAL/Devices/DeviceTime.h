/*
 * These methods are used for devices that replay sensor information (ie. logs) and are used to
 * synchronize multiple devices through timestamps. The QUEUE holds the _next_ timestamp of the
 * sensor reading. The QUEUE is ordered non-decreasing, so whenever the time inserted is on top
 * of the QUEUE, then the sensor can deliver the reading.
 *
 * The general template to use this is as follows:
 *
 * - On the sensor's INIT function, read ahead the upcoming event's timestamp and use PushTime()
 *   to push it to the QUEUE.
 *
 * - In the CAPTURE method, lock_wait() until the timestamp on top of the QUEUE [accessed via
 *   NextTime()] matches the one you pushed. If it does, give reading to user and call
 *   PopAndPushTime() which will pop your time from the QUEUE and push in the NEW time of your
 *   NEXT event.
 *
 */

#pragma once

namespace hal {
namespace DeviceTime {

void ResetTime();

/// Get time on top of queue
double NextTime();

/// Sleep until our time (nextTime) is next up.
void WaitForTime(double nextTime);

/// Push time on top of queue
void PushTime( double T );

/// Pop top of queue and push new time
void PopAndPushTime( double T );

/// Pop time on top of queue. This should never be used unless in some really really special
/// circumstances. Use PopAndPushTime() instead!
void PopTime();

/// Specify whether events should be played back in realtime
void SetRealtime(bool realtime=true);

/// Pause virtual time to stop new events
void PauseTime();

/// Unpause virtual time to resume receiving events
void UnpauseTime();

/// Toggle between playing and pausing virtual time.
void TogglePauseTime();

/// Allow numEvents events to pass before pausing
void StepTime(int numEvents);

}
}
