#!/bin/sh

echo ''
echo 'UPDATING THE EC2...'
echo ''
echo ''
sudo yum -y update

echo ''
echo 'PREPARING TO INSTALL JAVA...'
echo ''
echo ''
sudo amazon-linux-extras enable corretto8
sudo yum clean metadata


echo ''
echo 'INSTALLING JAVA...'
echo ''
echo ''
sudo yum install -y java-1.8.0-amazon-corretto

echo ''
echo 'INSTALLING DISPLAY SERVER (vncserver)...'
echo ''
echo ''
sudo yum install -y tigervnc-server

echo ''
echo 'INSTALLING SatelliteSimulator...'
echo ''
echo ''
aws s3 cp s3://romgeralesatellite/satellitesimulator-0.2.0-SNAPSHOT-executable.zip .
aws s3 cp s3://romgeralesatellite/runMultiSimulation.sh .
aws s3 cp s3://romgeralesatellite/runMultiSimulationUncertaintyStructured.sh .
aws s3 cp s3://romgeralesatellite/runMultiSimulationUncertaintyUnstructured.sh .
unzip satellitesimulator-0.2.0-SNAPSHOT-executable.zip
chmod 777 *.sh

echo ''
echo 'CONFIGURING vncserver...'
echo ''
echo ''
vncserver

echo ''
echo 'INSTALLATION CONCLUDED!'
echo ''
echo ''
