.PHONY: liveView test

# deploy-hostname: build and deploy sensor_fusion on SD card
deploy-%:
	NAME=$*  rebar3 grisp deploy 
screen:
	sudo picocom /dev/ttyUSB1 --baud 115200 --echo 

# remote-hostname: open a remote shell connected to hostname
remote-%:
	erl -sname remote_$* -remsh body_head@$* -setcookie MyCookie -kernel net_ticktime 8
