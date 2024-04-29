.PHONY: liveView test

# deploy-hostname: build and deploy sensor_fusion on SD card
deploy-%:
	NAME=$*  rebar3 grisp deploy 
screen:
	sudo picocom /dev/ttyUSB1 --baud 115200 --echo 

# remote-hostname: open a remote shell connected to hostname
remote-%:
	erl -sname remote_$* -remsh body_head@$* -setcookie MyCookie -kernel net_ticktime 8
# run_local: start sensor_fusion in release mode (clean start)
run_local:
	./_build/computer/rel/body_head/bin/body_head console

# local_release: build sensor_fusion in release mode for the computer
local_release:
	rebar3 as computer release

make clean:
	rm -rf measures/*
make vis:
	python3 Visualisation.py
make shell:
	rebar3 as computer shell --sname body_head --setcookie MyCookie