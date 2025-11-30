# OpenHRStrap
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)
![Status](https://img.shields.io/badge/Status-Prototype-orange)
![Platform](https://img.shields.io/badge/Platform-ESP32-blue)
![Language-Python](https://img.shields.io/badge/Language-Python-blue)
![Language-C++](https://img.shields.io/badge/Language-C++-blue)

OpenHRStrap is an open-source DIY chest-strap heart-rate tracker built around the ESP32. It measures real biosignals through electrodes, filters the signal, and applies the Pan–Tompkins algorithm to detect R-peaks and compute heart rate in real time. The Pan-Tompkins algorithm has been implemented in Python to follow the filter chain as originally described in the paper. The device is programmed so that it is detected as an off-the-shelf BLE device and is recognized by apps like Strava! Below, I will also show how you can extract data from your Strava and analyze it using Python!

![First Version of OpenHRStrap](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/CroppedFinishedProject.png)

Latest Blog on the project: [element14 Running Tracker Blog](https://community.element14.com/challenges-projects/element14-presents/project-videos/w/documents/71995/build-your-own-esp32-fitness-heart-rate-monitor-tracker----episode-692?ICID=HP-E14P-EP692-FITNESS-HEART-RATE-MONITOR-NOV25)

Link to the latest video on the project: [Running Tracker Video Part 1](https://www.youtube.com/watch?v=Z1Dts_NHXyQ)

Original Pan-Tompkins Paper: [A Real-Time QRS Detection Algorithm](https://www.robots.ox.ac.uk/~gari/teaching/cdt/A3/readings/ECG/Pan+Tompkins.pdf)

## Table of Contents

- [Project Updates](#project-updates)
- [V1 of the Project](#v1-of-the-project)
- [Pan–Tompkins Algorithm](#pan\-tompkins-algorithm)
- [Data Filtering Example](#data-filtering-example)
- [Detection Example](#detection-example)
- [Strava Data](#strava-data)
- [Disclaimer](#disclaimer)

## Project Updates

November 2025 — DIY-friendly version made using an XIAO ESP32 board with the AD8232 ECG module. Pan–Tompkins integration tested on static data works great; real-time detection while running is still a WIP. Scroll below for more details!  

V2 is currently in progress! This will include a custom PCB with proper data logging and an upgraded AFE. If you have any suggestions for what else you would like to see added, feel free to reach out!

## V1 of the Project
The text below covers the V1 of this project. Any new updates on the project after V1 will be placed above this. In this project, I wanted to create a DIY open-source version of a running chest strap sensor. My goals for this project were:

1. Module that fits on a commercial chest strap
2. Measures the ECG signal and calculates the heart rate from it
3. Connects to the phone over BLE like a commercial device

### Hardware
The hardware for this version is extremely DIY friendly and is made out of commonly found modules. As an MCU, it relies on an ESP32 XIAO S3 board since it has more than enough power to process the signal, and the BLE is easy to work with. For the analogue front end (AFE), I went with the most popular ECG module, the AD8232. This meant the chest strap needed another electrode added to the chest strap, which I did by sewing an additional button to the back of the strap for a reference electrode. The case for this device was printed out of PLA and TPU, and to interface with the chest strap, I soldered wires directly to steel push buttons. Besides that, you will need a way to power it. I repurposed a small 1S LION cell I had lying around with a small boost converter that stepped up the voltage to 5V; the onboard ESP electronics handled the rest.

![Schematics](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Schematic%20V1.PNG)

![Hardware Drawing](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Drawings/Drawing_DeviceDesign.PNG)

### Pan-Tompkins Algorithm
For calculating the heart rate, I wanted to go with the Pan-Tompkins algorithm, since it is one of the most significant and widely used methods for ECG signal processing. It originates from a paper titled “A Real-Time QRS Detection Algorithm” that was published by Jiapu Pan and Willis J. Tompkins in 1985. The full paper can be viewed for free on the following link: [A Real-Time QRS Detection Algorithm](https://www.robots.ox.ac.uk/~gari/teaching/cdt/A3/readings/ECG/Pan+Tompkins.pdf)

In the first video that covers the progress of this project, I covered in detail how the whole algorithm works: [Running Tracker Video Part 1](https://www.youtube.com/watch?v=Z1Dts_NHXyQ)

I will here briefly explain how the Pan-Tompkins algorithm works with some illustrations and sample data, but I also highly recommend reading through the original paper! The original name of the algorithm now known as the Pan-Tompkins algorithm is the algorithm for real-time QRS detection, and its purpose is to recognise a QRS complex in the signal, or simply said, a heartbeat. The algorithm works by keeping track of a few key parameters:

1. SPK - Running Estimate of the Signal Peak
2. NPK - Running Estimate of the Noise Peak
3. TH1 - Threshold 1 for detecting peaks
4. TH2 - Threshold 2 for detecting peaks, usually TH1 / 2
Each of these has variants for both the filtered signal (“F”) and the fully processed integrated signal (“I”).

At the start, all of the parameters are initiated on a few seconds of data that won't be used, by looking at the maximum peak value from that part of the signal, or by looking at the mean and maximum values of the signal. After the initialisation, the signal is analysed in small batches where it is first run through a series of filters and peak detections. Tompkins filtering pipeline:

1. Band-pass filtering
2. Derivative
3. Squaring
4. Moving window integration

This produces two sets of peaks:
1. FILT peaks – peaks from the band-pass filtered signal
2. INT peaks – peaks from the fully processed integrated signal 

Going with the INT peaks, the peaks are analysed to check whether they satisfy the condition of being above the value of TH1. If this is the case for a peak, we then search for a peak in FILT peaks that also satisfies its TH1 and is within a small delta t of the INT peak. Only when we find that FILT peak, we can classify that as an R peak and update our running parameters based on the peak value (SPK, NPK, TH1, TH2). If, however, a peak is in any case under TH1, it is classified as a noise peak, and the running parameters are updated with different formulas. What makes Pan-Tompkins such a good algorithm is the search-back part, which is activated if no R peak has been classified for more than 166% of the RR average distance. This means we missed a peak due to noise or some other issue, and the algorithm starts the search back from the last classified R peak, but this time, using the TH2 to categorise a peak as an R peak. The above explains the essence of the algorithm with some more important things like RRR distance tracking, explained in more detail in the drawing and in the video!

![Pan-Tompkins Drawing](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Drawings/Drawing_PanTompkins.PNG)

#### Data Filtering Example
In this section, I will show an example of the Pan-Tompkins algorithm working on some data I collected using the setup described above. All of the codes, both for the data acquisition and for data analysis, can be found in the GitHub repo and on the [element14 community](https://community.element14.com/challenges-projects/element14-presents/project-videos/w/documents/71995/build-your-own-esp32-fitness-heart-rate-monitor-tracker----episode-692?ICID=HP-E14P-EP692-FITNESS-HEART-RATE-MONITOR-NOV25). If you believe anything is missing, feel free to contact me!

##### Raw ECG Signal
To begin, let's first take a look at the raw ECG signal and a segment of that signal that we will be looking out throughout the whole analysis.

![Full ECG Signal](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Figures/Fig1_FullECG.png)

![ECG Segment](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Figures/Fig3_SegmentECG.png)

##### Band-Pass Filter
The signal is first passed through a band-pass filter. To accomplish this, and to follow the paper, this was implemented as a low-pass filter followed by a high-pass filter. The results of that can be seen below.

![Low-Pass](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Figures/Fig4_LowPassECG.png)

![Band-Pass](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Figures/Fig5_BandPassECG.png)

##### Derivative Filter
The signal seen above is the one on which we will detect the peaks, but we also need to filter it further to get to our "INT" signal. The next filter we are passing our signal through is the derivative filter; the results of that are shown below.

![Derivative Filter](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Figures/Fig6_DerivativeECG.png)

##### Squaring
After the derivative filter, the signal is squared

![Squaring](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Figures/Fig7_SquaredECG.png)

##### Moving Window Integration
The last filter we will pass our signal through before running the peak detection algorithm is the Moving Window Integrator.

![MWI](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Figures/Fig8_MWIECG.png)

##### INT Peaks
Running the peak detection algorithm on a signal with not too much noise, we will usually grab all of the R peaks, some T waves, and a few noise peaks. Example of that below.

![INT Peaks](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Figures/Fig9_IntPeaks.png)

##### FILT Peaks
We also run the same peak detection algorithm on the band-pass filtered signal. That signal and the detected peaks are shown in the figure below.

![FILT Peaks](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Figures/Fig10_FiltPeaks.png)

#### Detection Example
Since this was pre-recorded data, all of the filters and algorithms have been applied to the data at once, but the detection itself works essentially in the same way whether it goes in small batches or like this, since it iterates through the peaks. A short segment showing the detection on both INT and FILT signals is shown in the first two figures below.

![INT Detection](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Figures/Fig12_Int_FullPT.png)

![FILT Detection](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Figures/Fig14_Filt_FullPT.png)

In the two figures below, you can see the results from the whole dataset. It worked rather nicely! I can see one missed R peak on 80+ seconds, see if you can spot it as well! The peaks weren't detected in the first 10 seconds of the algorithm, since that part of the signal was used for initializing the running parameters!

![INT Detection](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Figures/peak_det1.png)

![FILT Detection](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Figures/peak_det2.png)

### Strava Data
This section will focus on the real-world testing of this device. This is where the V1 ran into some issues. The test was conducted by doing a 5K run with the chest strap and while wearing a smart watch with a PPG sensor on the arm as a control device. Smartwatches with PPG sensors are notorious for not being as accurate as ECG chest straps for running, but I still needed a reference device for the tests. I run long enough to approximately know whether the pulse reading is in the ball park, and on this case, there weren't any issues with the smart watch. The data that you can see below has been downloaded from Strava. If you log into Strava on their website and go to your activities, you can download the activity file and analyse it using Python! Code for that is also available in this repo. The two figures below show the data recorded by the smartwatch and by our DIY device.

![SmartWatch Data](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Strava%20Data/Fig1_Strava_Smartwatch.png)

![DIY Device Data](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Strava%20Data/Fig2_Strava_ESP32HR.png)

The Smartwatch PPG signal is heavily filtered and is not realistic in that sense, but the measurement from our DIY device should follow the same trends as the PPG signal, and the values should be in the ballpark. If we overlay the signals one on top of the other by using the timestamps to align them, we can more easily compare them. This is shown in the figure below.

![Comparisson](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Strava%20Data/Fig3_Strava_Comparisson.png)

We can see that at the very start, as I started my run, both devices really matched nicely one another, bubt as my heartrate went up, so did the gap between the signals... If we take a look futher down the signal, we can see that as I tried changing my tempo during the run, the signals would sometimes match rather nicely when my heart rate would go down, but it was also way off in some other cases. It is also clear that, for the most part, the heart rate recorded by our DIY device was lower than the smartwatch. Since I didn't have proper data logging to see how the signal itself looked and what the algorithm did, it's hard to say what the main reason was for this. In the figures below, you can see segments of the signal where they were close together and when they were close together and then split up.

![comp1](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Strava%20Data/Fig4_Strava_GoodSegment.png)

![comp2](https://github.com/MilosRasic98/OpenHRStrap/blob/main/Pictures/Strava%20Data/Fig5_Strava_BadSegmentpng.png)

The current plan for this project is to design a PCB around a new AFE IC and add datalogging capabilities. This will greatly increase the troubleshooting with real world testing and tuning the algorithm and hardware so that it is reliabble as an off-the-shelf device while keeping the whole thing completely open-source.

## Disclaimer

This project is intended for **educational, experimental, and hobby use only**.  
OpenHRStrap is **not a medical device**, is **not certified**, and must not be used for diagnosis, monitoring of medical conditions, or any health-critical applications.  

All hardware, firmware, and software are provided **“as-is” without warranty**, and the author assumes **no responsibility** for any damages, injuries, or losses resulting from the use of this project.

