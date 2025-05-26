#!/bin/bash
# Script to configure persistent Arduino symlinks for Tino Robot
# Run this once for each Arduino to identify and set up symlinks

echo "Arduino Persistent Device Name Setup for Tino Robot"
echo "=================================================="
echo ""
echo "This script will help you set up persistent device names for your Arduino boards."
echo "Please connect only ONE Arduino at a time and follow the instructions."
echo ""

# Function to reset all rules
reset_rules() {
    if [ -f /etc/udev/rules.d/99-tino-arduino.rules ]; then
        echo "Deleting all existing Arduino udev rules..."
        sudo rm /etc/udev/rules.d/99-tino-arduino.rules
        echo "Rules file deleted."
    else
        echo "No existing rules file found."
    fi
    
    # Create empty rules file
    echo "# Tino Robot Arduino udev rules" | sudo tee /etc/udev/rules.d/99-tino-arduino.rules > /dev/null
    echo "Created fresh rules file."
}

# Check for --reset flag
if [ "$1" = "--reset" ]; then
    reset_rules
    echo "All Arduino udev rules have been reset. Run this script again without --reset to set up your devices."
    exit 0
fi

echo "Tip: Run with --reset to delete all existing rules before setting up devices."
echo ""

# Function to get Arduino serial number
get_serial_number() {
    local dev_path=$1
    if [ -e "$dev_path" ]; then
        vendor=$(udevadm info -a -n "$dev_path" | grep -m 1 '{idVendor}' | cut -d '"' -f 2)
        product=$(udevadm info -a -n "$dev_path" | grep -m 1 '{idProduct}' | cut -d '"' -f 2)
        
        # Get serial number - check if it's a CH341 device
        if [ "$vendor" == "1a86" ] && [ "$product" == "7523" ]; then
            # For CH341 adapters, we'll use a combination of USB port path and vendor/product ID
            # This should be more reliable than the default serial which is often empty or a PCI path
            port_path=$(udevadm info -a -n "$dev_path" | grep -m 1 'ATTRS{devpath}' | cut -d '"' -f 2)
            serial="CH341-$port_path"
            
            echo "Found CH341 USB-Serial adapter with:"
            echo "  Port Path: $port_path"
            echo "  Using generated serial: $serial"
        else
            # For regular Arduino devices
            serial=$(udevadm info -a -n "$dev_path" | grep -m 1 '{serial}' | cut -d '"' -f 2)
            
            if [ -z "$serial" ]; then
                # No serial found, use the USB port path instead
                port_path=$(udevadm info -a -n "$dev_path" | grep -m 1 'ATTRS{devpath}' | cut -d '"' -f 2)
                serial="USB-$port_path"
                echo "No serial found, using USB port path as identifier."
            fi
        fi
        
        if [ -n "$serial" ]; then
            echo "Found device with:"
            echo "  Serial: $serial"
            echo "  Vendor ID: $vendor"
            echo "  Product ID: $product"
            return 0
        fi
    fi
    echo "No device found at $dev_path."
    return 1
}

# Function to add udev rule
add_udev_rule() {
    local serial=$1
    local vendor=$2
    local product=$3
    local device_type=$4
    local symlink=$5

    # Special handling for CH341 devices
    if [[ "$serial" == "CH341-"* ]]; then
        # Extract the port path from our generated serial
        port_path=${serial#CH341-}
        # For CH341, use devpath instead of serial which is more reliable
        rule="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vendor\", ATTRS{idProduct}==\"$product\", ATTRS{devpath}==\"$port_path\", SYMLINK+=\"$symlink\", GROUP=\"dialout\", MODE=\"0666\""
    else
        # Standard rule for devices with serial numbers
        rule="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vendor\", ATTRS{idProduct}==\"$product\", ATTRS{serial}==\"$serial\", SYMLINK+=\"$symlink\", GROUP=\"dialout\", MODE=\"0666\""
    fi
    
    echo "Setting up rule for $device_type Arduino ($symlink):"
    echo "$rule"
    
    # Check if rules file exists
    if [ -f /etc/udev/rules.d/99-tino-arduino.rules ]; then
        # Delete any existing rule for this device (by serial)
        if grep -q "$serial" /etc/udev/rules.d/99-tino-arduino.rules; then
            echo "Existing rule found for this device. Removing it first..."
            sudo sed -i "/$serial/d" /etc/udev/rules.d/99-tino-arduino.rules
        fi
        
        # Also delete any rule for the same symlink (regardless of serial)
        if grep -q "$symlink" /etc/udev/rules.d/99-tino-arduino.rules; then
            echo "Existing rule found for symlink $symlink. Removing it..."
            sudo sed -i "/$symlink/d" /etc/udev/rules.d/99-tino-arduino.rules
        fi
        
        # Handle case where serial might be a PCI path (common with CH341 adapters)
        if [[ "$serial" == *":"* ]]; then
            echo "Detected PCI path-style serial. Ensuring proper escaping for sed..."
            # We need to escape the serial for sed since it contains special characters
            escaped_serial=$(echo "$serial" | sed 's/[\/&]/\\&/g')
            sudo sed -i "/$escaped_serial/d" /etc/udev/rules.d/99-tino-arduino.rules
        fi
    fi
    
    # Now add the new rule
    echo "$rule" | sudo tee -a /etc/udev/rules.d/99-tino-arduino.rules > /dev/null
    echo "Rule added successfully."
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
echo "Which device is this?"
echo "1) Motor Arduino (ttyMOTOR)"
echo "2) Sensor Arduino (ttySENSE)"
echo "3) Power Supply Arduino (ttyPOWER)"
echo "4) LIDAR Sensor (ttyLIDAR)"
read -p "Select [1-4]: " choice

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
    4)
        add_udev_rule "$serial" "$vendor" "$product" "LIDAR" "ttyLIDAR"
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
echo "After connecting all devices, you should have the following device links:"
echo "- Motor Arduino: /dev/ttyMOTOR"
echo "- Sensor Arduino: /dev/ttySENSE"
echo "- Power Supply Arduino: /dev/ttyPOWER"
echo "- LIDAR Sensor: /dev/ttyLIDAR"
echo ""
echo "These device names will persist across reboots."
echo ""