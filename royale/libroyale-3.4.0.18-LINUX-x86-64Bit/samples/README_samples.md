Royale sample applications
==========================

Some of these samples are only provided in the relevant platform's SDK, for example the Android
example is omitted from the other platforms' SDKs.

C++ examples
============

sampleCameraInfo
----------------

This C++ example shows how to create a camera and query information about the camera.  It doesn't
capture any images.

This sample can also be a useful tool, it prints the list of supported modes (which can be used with
Royaleviewer's `--mode` option), and for UVC devices it prints the UVC firmware version.

sampleRetrieveData
------------------

This C++ example shows how to capture image data.

It's a command line application that does not depend on any GUI toolkit, therefore it only displays
textual information about the captured images.

sampleExportPLY
---------------

This C++ example shows how to playback recorded files and export the data to PLY.

A command line application that takes a rrf filename as input and outputs PLY files for every
frame of the recording into the current working directory.

sampleOpenCV
------------

This C++ example shows how to capture image data, fill OpenCV images and display the data with HighGUI.
It was tested with OpenCV 2.4.13.

To compile this you need to point CMake to your OpenCV installation folder.

C example
=========

We recommend reading the C++ examples before reading the C example.

sampleCAPI
----------

This C example shows how to query information about the camera and capture data.

It's equivalent to a combination of the functionality of the C++ sampleCameraInfo and sampleRetrieveData.

C# example
==========

sampleDotNet
------------

This C# example shows how to capture data on Microsoft's .NET framework.

It's a command line application without a GUI part, therefore it only displays textual information
about the captured data.

Android example
===============

android
-------

This JNI example (both C++ and Java) shows how to use a USB camera on Android phones that support
acting as USB hosts.

The data is received in a C++ callback, and the sample shows how to pass the received data to Java
in the MainActivity.amplitudeCallback().

Examples for specific requirements
==================================

sampleMasterSlave
---------------------

This C++ sample uses two synchronised cameras, which need to be physically connected via an external
trigger cable.

The pico maxx and pico monster cameras support acting as either master or slave (the trigger cable
plugs in to the non-USB external connector on these cameras).

Utility
=======

inc/sample\_utils
-----------------

This contains the PlatformResources utility required for the C and C++ examples to use some cameras
on some platforms. Currently it's only required for UVC cameras on Windows, as the media framework
requires the application to set up the COM environment before calling the library code.
