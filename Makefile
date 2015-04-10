all: mavlink_control

mavlink_control: mavlink_control.cpp
	g++ -I mavlink/include/mavlink/v1.0 mavlink_control.cpp serial_port.cpp autopilot_interface.cpp -o laika -lpthread

clean:
	 rm -rf *o laika
