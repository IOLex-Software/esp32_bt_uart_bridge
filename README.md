
Esp32 bt to uart bridge

Send data received via bt in spp mode to one of uarts and vice versa.

Note:
1. Had a problem with powering my esp from my laptop. Couldn't esatblished bt connection. Apparently it doesn't supply enough power. Plugging esp32 to desktop PC solved problem.
2. Communication from pc to esp32 gives transmission errors. Ratio depends on distance. Average 1% of data has an error. Transmission in other direction is much better.
