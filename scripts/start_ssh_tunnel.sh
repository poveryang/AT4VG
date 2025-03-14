#!/bin/bash
# Start SSH Tunnel
ssh -L 8080:192.168.100.56:8080 win -N -f
echo "SSH tunnel started: localhost:888 -> 192.168.100.56:22"