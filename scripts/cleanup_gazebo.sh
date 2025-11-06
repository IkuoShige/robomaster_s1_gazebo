#!/bin/bash
################################################################################
# Gazebo Cleanup Script
#
# This script safely stops all Gazebo processes and cleans up resources
################################################################################

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Gazebo Cleanup Script${NC}"
echo -e "${BLUE}========================================${NC}\n"

# Check for running Gazebo processes
echo -e "${BLUE}Checking for running Gazebo processes...${NC}"
GAZEBO_PROCESSES=$(ps aux | grep -E "gazebo|gzserver|gzclient" | grep -v grep | wc -l)

if [ "$GAZEBO_PROCESSES" -eq 0 ]; then
    echo -e "${GREEN}No Gazebo processes found.${NC}"
    exit 0
fi

echo -e "${YELLOW}Found $GAZEBO_PROCESSES Gazebo process(es):${NC}"
ps aux | grep -E "gazebo|gzserver|gzclient" | grep -v grep | awk '{print "  PID " $2 ": " $11 " " $12 " " $13}'

# Ask for confirmation
echo ""
read -p "Kill all Gazebo processes? [y/N] " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}Cleanup cancelled.${NC}"
    exit 0
fi

# Kill processes
echo -e "${BLUE}Killing Gazebo processes...${NC}"

# Try graceful shutdown first
pkill -15 gzclient 2>/dev/null
pkill -15 gzserver 2>/dev/null
pkill -15 gazebo 2>/dev/null

sleep 2

# Force kill if still running
REMAINING=$(ps aux | grep -E "gazebo|gzserver|gzclient" | grep -v grep | wc -l)
if [ "$REMAINING" -gt 0 ]; then
    echo -e "${YELLOW}Some processes still running. Force killing...${NC}"
    pkill -9 gzclient 2>/dev/null
    pkill -9 gzserver 2>/dev/null
    pkill -9 gazebo 2>/dev/null
    sleep 1
fi

# Verify cleanup
FINAL_COUNT=$(ps aux | grep -E "gazebo|gzserver|gzclient" | grep -v grep | wc -l)
if [ "$FINAL_COUNT" -eq 0 ]; then
    echo -e "${GREEN}All Gazebo processes stopped successfully.${NC}"
else
    echo -e "${RED}Warning: Some processes may still be running.${NC}"
    ps aux | grep -E "gazebo|gzserver|gzclient" | grep -v grep
fi

# Clean up shared memory
echo -e "${BLUE}Cleaning up shared memory...${NC}"
ipcs -m | grep $USER | awk '{print $2}' | xargs -r ipcrm -m 2>/dev/null || true

echo -e "${GREEN}Cleanup complete!${NC}"
