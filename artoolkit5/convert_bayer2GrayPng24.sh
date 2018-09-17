#!/bin/bash
FORMAT=${1}
for i in $(ls *${FORMAT}); do
  convert ${i} -define showkernel=1 \
          -morphology Convolve '3x3: 0.0625, 0.125, 0.0625
                                     0.125,  0.25,  0.125
                                     0.0625, 0.125, 0.0625' PNG24:${i}.png;
done
