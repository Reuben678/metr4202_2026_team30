cd ~/METR4202/metr4202_2026_team30

# Remove old shit
rm -rf /install /build /log

# Rebuild
cd src/
colcon build --packages-select metr4202_interfaces project_explore project_search

source install/setup.bash
# Ready to run
cd ..
echo "Rebuild done"