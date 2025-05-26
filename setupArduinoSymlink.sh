#!/bin/bash
# Script to configure persistent Arduino symlinks for Tino Robot
# Run this once for each Arduino to identify and set up symlinks

echo "Arduino Persistent Device Name Setup for Tino Robot"
echo "=================================================="
echo ""
echo "This script will help you set up persistent device names for your Arduino boards."
echo "Please connect only ONE Arduino at a time and follow the instructions."
echo ""

# Function to get Arduino serial number
get_serial_number() {
    local dev_path=$1
    if [ -e "$dev_path" ]; then
        serial=$(udevadm info -a -n "$dev_path" | grep -m 1 '{serial}' | cut -d '"' -f 2)
        vendor=$(udevadm info -a -n "$dev_path" | grep -m 1 '{idVendor}' | cut -d '"' -f 2)
        product=$(udevadm info -a -n "$dev_path" | grep -m 1 '{idProduct}' | cut -d '"' -f 2)
        
        if [ -n "$serial" ]; then
            echo "Found Arduino with:"
            echo "  Serial: $serial"
            echo "  Vendor ID: $vendor"
            echo "  Product ID: $product"
            return 0
        fi
    fi
    echo "No Arduino device found at $dev_path."
    return 1
}

# Function to add udev rule
add_udev_rule() {
    local serial=$1
    local vendor=$2
    local product=$3
    local device_type=$4
    local symlink=$5

    rule="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vendor\", ATTRS{idProduct}==\"$product\", ATTRS{serial}==\"$serial\", SYMLINK+=\"$symlink\", GROUP=\"dialout\", MODE=\"0666\""
    
    echo "Adding rule for $device_type Arduino ($symlink):"
    echo "$rule"
    
    # Check if the rule already exists
    if grep -q "$serial" /etc/udev/rules.d/99-tino-arduino.rules; then
        echo "Rule for this device already exists. Skipping."
    else
        echo "$rule" | sudo tee -a /etc/udev/rules.d/99-tino-arduino.rules > /dev/null
        echo "Rule added successfully."
    fi
}

# Main script
echo "Checking for Arduino device (ttyACM*)..."

# Find the first available /dev/ttyACM* device
dev_path=""
for dev in /dev/ttyACM*; do
    if [ -e "$dev" ]; then
        dev_path="$dev"
        break
    fi
done

if [ -z "$dev_path" ]; then
    # Fallback to ttyUSB if no ACM device found
    for dev in /dev/ttyUSB*; do
        if [ -e "$dev" ]; then
            dev_path="$dev"
            break
        fi
    done
fi

if [ -z "$dev_path" ]; then
    echo "No Arduino device found. Please connect an Arduino and try again."
    exit 1
fi

echo "Arduino found at $dev_path"
echo ""

# Get serial number
get_serial_number "$dev_path"
if [ $? -ne 0 ]; then
    echo "Failed to get serial number. Please check the connection."
    exit 1
fi

echo ""
echo "Which Arduino is this?"
echo "1) Motor Arduino (ttyMOTOR)"
echo "2) Sensor Arduino (ttySENSE)"
echo "3) Power Supply Arduino (ttyPOWER)"
read -p "Select [1-3]: " choice

case $choice in
    1)
        add_udev_rule "$serial" "$vendor" "$product" "Base" "ttyMOTOR"
        ;;
    2)
        add_udev_rule "$serial" "$vendor" "$product" "Leg" "ttySENSE"
        ;;
    3)
        add_udev_rule "$serial" "$vendor" "$product" "PowerSupply" "ttyPOWER"
        ;;
    *)
        echo "Invalid selection."
        exit 1
        ;;
esac

echo ""
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "Setup complete!"
echo "After connecting all Arduinos, you should have the following device links:"
echo "- Motor Arduino: /dev/ttyMOTOR"
echo "- Sensor Arduino: /dev/ttySENSE"
echo "- Power Supply Arduino: /dev/ttyPOWER"
echo ""
echo "These device names will persist across reboots."
echo ""