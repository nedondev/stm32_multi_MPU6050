# stm32_multi_MPU6050
STM32 read MPU6050 sensor via TCA9548A I2C multiplexer and return data via UART
	
pre-install:

python-socketio 
	- pip3 install python-socketio
uvicorn 
	- pip3 install uvicorn
python-dotenv 
	- pip3 install python-dotenv

For window user that use python in anaconda might face problem to install uvicorn.
solve by : 
	- conda install -c conda-forge uvicorn
ref : https://anaconda.org/conda-forge/uvicorn
