# SimonXT-RF-Monitor
Arduino/ESP32 project for decoding Interlogix (ITI) RF sensor packets with a CC1101 receiver, logging events to syslog, and providing real‑time LED feedback for door contacts and motion sensors. Includes supervisor filtering, two‑event confirmation logic for reliable state changes, and independent LED timers to prevent race conditions.


Current Bugs are that is seldomly will log ghost events and flip the logic can be fixed by increasing the timer on if (liveCache[cacheIdx].pendingConfirmCount >= 3) to 3 in place of 2 again this causes issues with the actual events it will seldomly miss them. But you can clearly see when doors open and close if they are normal events you can see the logs open and close next to one another 
