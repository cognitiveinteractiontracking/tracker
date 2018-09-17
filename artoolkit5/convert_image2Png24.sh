#!/bin/bash
FORMAT=${1}
for i in $(ls *${FORMAT}); do
  convert ${i} PNG24:${i}.png;
done
