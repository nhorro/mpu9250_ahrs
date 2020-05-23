# AHRS MPU9250 library

Arduino code and jupyter notebooks of AHRS with MPU9250.

Project organization:

~~~
./
	arduino 	
		mpu9250_capture		Print CSV raw and processed measurements (can be used with Arduino Plotter).
		mpu9250_ahrs.h      AHRS MPU9250 library.
	data 					CSV captures.
	notebooks				Notebooks with AHRS algorithm development.
	doc						MPU9250 datasheet and other references.
	python					Simple script to save CSVs.
~~

[Detailed description of the algorithm implementation](notebooks/01%20-%20BasicAHRS.ipynb)