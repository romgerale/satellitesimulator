#!/bin/sh

cd satellitesimulator-0.2.0-SNAPSHOT

vncserver
export DISPLAY=:1

(java -Dorekit.data.path=./orekit-data -Xms1G -Xmx4G -cp lib/satellitesimulator-0.2.0-SNAPSHOT.jar br.inpe.cmc202.simulation.MultiSimulationController 1000 1 1)

cd ..

aws s3 cp satellitesimulator-0.2.0-SNAPSHOT s3://romgeralesatellite/images --recursive --exclude "*" --include "*.png"
aws s3 cp satellitesimulator-0.2.0-SNAPSHOT s3://romgeralesatellite/images --recursive --exclude "*" --include "*.log"

sudo shutdown
