# OpenHRStrap
OpenHRStrap is an open-source DIY chest-strap heart-rate tracker built around the ESP32. It measures real biosignals through electrodes, filters the signal, and applies the Pan–Tompkins algorithm to detect R-peaks and compute heart rate in real time. The Pan-Tompkins algorithm has been implemented in Python to follow the filter chain as originally described in the paper. 

Latest Blog on the project: [element14 Running Tracker Blog]([url](https://community.element14.com/challenges-projects/element14-presents/project-videos/w/documents/71995/build-your-own-esp32-fitness-heart-rate-monitor-tracker----episode-692?ICID=HP-E14P-EP692-FITNESS-HEART-RATE-MONITOR-NOV25))

Link to the latest video on the project: [Running Tracker Video Part 1]([url](https://www.youtube.com/watch?v=Z1Dts_NHXyQ))

Original Pan-Tompkins Paper: [A Real-Time QRS Detection Algorithm]([url](https://www.robots.ox.ac.uk/~gari/teaching/cdt/A3/readings/ECG/Pan+Tompkins.pdf))


## Project Updates

November 2025 - DIY Friendly version made using an XIAO ESP32 board with the AD8232 ECG module. Pan-Tompkins integration that was tested on static data and works great, with real data, while running, still a WIP, scroll below for more details!
V2 is in progress!

## Introduction

