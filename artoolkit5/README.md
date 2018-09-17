# artoolkit5

* Build artoolkit5-5.3.3 with in `/opt/artoolkit5` with exclusive `gstreamer 0.10`
  * `cd /opt && git clone https://github.com/artoolkit/artoolkit5.git artoolkit5-5.3.3 && cd artoolkit5-5.3.3 && git checkout 68523ea2e7503bc0fc56f31776185af2cb794c31`
  * `\Configure`
  * `make -j4`
  * `cd share && ./artoolkit5-setenv && echo "export ARTOOLKIT5_ROOT=/opt/artoolkit5-5.3.3" >> ~/.bashrc`

## Print Marker

* Assuming the following setup
  * Ubuntu
  * cups: https://wiki.ubuntuusers.de/CUPS/
  * cups-pdf: https://wiki.ubuntuusers.de/CUPS-PDF/
  * Printername: ksPrinter
* Currently used marker for setup: https://github.com/artoolkit/artoolkit5/tree/master/doc/patterns/Matrix%20code%203x3%20with%20parity%20(72dpi)

* Hints
  * List Printer: lpstat -p -d
  * List Printer options: lpoptions -p <printer> -l

The common marker image has 72ppi, so if you print it with the given command, the marker (227x227 px) will become 227px/72ppi*2.54cm/inch=8cm in width, e.g.:

* 227px/72ppi*2.54cm/inch=8cm
* 227px/57.658ppi*2.54cm/inch=10cm
* 227px/57ppi*2.54cm/inch=10.1cm
* 227px/58ppi*2.54cm/inch=9.9cm
* 227px/23.0632ppi*2.54cm/inch=25cm

You can just addjust the size by the ppi argument: Desired size in centimeter X and target ppi Y

* Calculation: `2.54cm/inch*227px/X = Y`

### Print Commands

#### Directly to the printer

* Single: `lpr -P ksPrinter -o ppi=72 0.png`
* All: `for file in $(seq 0 31); do lpr -P ksPrinter -o ppi=72 ${file}.png; done`

#### Printing to PDF and then to printer

* All: `for file in $(seq 0 31); do lpr -P PDF -o PageSize=A5 -o ppi=72 ${file}.png; done`
* Unite to single PDF (former output usually located in ${HOME}/PDF):
  * `pdfunite $(seq 0 31 | sed s/$/\.pdf/g | tr "\n" " ") /tmp/unite.pdf`
  * `lpr -P PDF -o fit-to-page -o PageSize=A3 -o number-up=4 /tmp/unite.pdf`
  * `lpr -P ksPrinter -o PageSize=A3 -o InputSlot=Tray3 -o page-ranges=1-1 ${HOME}/PDF/unite.pdf`
* Print 4 markers with the size of 8cm each on a single DIN-A3 page on the printer ksPrinter
  * `for file in $(seq 0 3); do lpr -P PDF -o PageSize=A5 -o ppi=72 ${file}.png; done`
  * `pdfunite $(seq 0 3 | sed s/$/\.pdf/g | sed s@^@${HOME}\/PDF\/@g | tr "\n" " ") /tmp/unite.pdf`
  * `lpr -P PDF -o fit-to-page -o PageSize=A3 -o number-up=4 /tmp/unite.pdf`
  * `lpr -P ksPrinter -o PageSize=A3 -o InputSlot=Tray3 -o page-ranges=1-1 ${HOME}/PDF/unite.pdf`
* Print one big marker with edge size of 25 cm on single DIN-A3 page on the printer ksPrinter
  * `lpr -P ksPrinter -o PageSize=A3 -o InputSlot=Tray3 -o ppi=23.0632 1.png`
* Print 16 markers with the size of 8cm each on a single DIN-A1 page as PDF
  * `for file in $(seq 0 15); do lpr -P PDF -o PageSize=A5 -o ppi=72 ${file}.png; done`
  * `pdfunite $(seq 0 15 | sed s/$/\.pdf/g | sed s@^@${HOME}\/PDF\/@g | tr "\n" " ") /tmp/unite.pdf`
  * `lpr -P PDF -o fit-to-page -o PageSize=A1 -o number-up=16 /tmp/unite.pdf`
* Print 32 markers with the size of 8cm each on a single DIN-A0 page as PDF (Usage with "artoolkit5/doc/patterns/Matrix code 3x3 with parity (72dpi)/")
  * `for file in $(seq 0 31); do lpr -P PDF -o PageSize=A5 -o ppi=72 ${file}.png; done`
  * `pdfunite $(seq 0 15 | sed s/$/\.pdf/g | sed s@^@${HOME}\/PDF\/@g | tr "\n" " ") /tmp/unite1of2.pdf`
  * `pdfunite $(seq 16 31 | sed s/$/\.pdf/g | sed s@^@${HOME}\/PDF\/@g | tr "\n" " ") /tmp/unite2of2.pdf`
  * `lpr -P PDF -o fit-to-page -o PageSize=A1 -o number-up=16 /tmp/unite1of2.pdf`
  * `lpr -P PDF -o fit-to-page -o PageSize=A1 -o number-up=16 /tmp/unite2of2.pdf`
  * `pdfunite ${HOME}/PDF/unite1of2.pdf ${HOME}/PDF/unite2of2.pdf /tmp/unite.pdf`
  * `lpr -P PDF -o fit-to-page -o PageSize=A0 -o number-up=2 /tmp/unite.pdf`

## Current Calibration

### Setup

* For the given calibration pattern, the follwoing properties have been used: `./calib_camera -cornerx=11 -cornery=7 -pattwidth=60 -imagenum=10`
  * `cornerx`: Intersection alonge the long edge
  * `cornery`: Intersection alonge the short edge
  * `pattwidth`: With of one square in millimeter
  * `imagenum`: Number of images to take for calibration
* If the crosses turn red, hit space to use that image for calibration
* Images are currently stored under `alpia.techfak.uni-bielefeld.de/media/filestorage/camera_calibration/`
* Copy the generated `camera_para.dat` to `calib/camera<#ofCamera>twb_para.dat`
* Note: Less than 1 pixel error is OK. Error greater than 2 pixels indicates a poor calibration, and it should be abandoned and restarted.

### Reference

* https://artoolkit.org/documentation/doku.php?id=2_Configuration:config_video_capture_gstreamer
* https://artoolkit.org/documentation/doku.php?id=2_Configuration:config_camera_calibration

## New Calibration

### First test the setup by reading in a test image source:

* `./calib_camera`
  * pattern from the central lab: `./calib_camera -cornerx=11 -cornery=7 -imagenum=10 -pattwidth=60`

or export an own test source

* `export ARTOOLKIT5_VCONF='videotestsrc ! capsfilter caps=video/x-raw-rgb,width=1024,height=680,bpp=24,framerate=30/1 ! identity name=artoolkit sync=true ! fakesink'`
* `./calib_camera`

### Using webcam

* `export ARTOOLKIT5_VCONF='v4l2src device=/dev/video0 use-fixed-fps=false ! ffmpegcolorspace ! video/x-raw-rgb,bpp=24 ! identity name=artoolkit sync=true ! fakesink'`
* `./calib_camera`

### Using recorded images on the hard drive

* First, prepare the image names
  * Optional: Convert any image type to RGB 8-bit PNG: `convert_image2Png24.sh`
  * Optional: Convert bayer image to RGB 8-bit PNG: `convert_bayer2Png24.sh`
  * Link to conscutive names: `idx=0; for i in *png; do ln -fs ${i} frame$(printf %05d ${idx}).png; idx=$(( idx + 1 ));done`
* Create a video out of the image sequence and replay it in gstreamer ...
  * `ffmpeg -i frame%05d.png -c:v videoraw test.avi`
* ... or use the PNG files directly (with this example, once a second an image will be played back)
* `export ARTOOLKIT5_VCONF='multifilesrc location=/tmp/images/cam1/frame%05d.png caps="image/png,framerate=\(fraction\)1/1,pixel-aspect-ratio=1/1" ! pngdec ! ffmpegcolorspace ! video/x-raw-rgb,bpp=24 ! videoscale ! videorate ! identity name=artoolkit sync=true ! fakesink'`
* `./calib_camera`
