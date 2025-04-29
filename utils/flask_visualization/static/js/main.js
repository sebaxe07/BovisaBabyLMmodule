// Initialize plot and variables
const plotDiv = document.getElementById('plot');
const statusDiv = document.getElementById('status');
const currentModeSpan = document.getElementById('current-mode');

let updateTimeout = null;
const THROTTLE_DELAY = 100; // milliseconds

// Initialize map
let map = null;
let robotMarker = null;
let geofenceCircle = null;
let followRobot = true;
let mapInitialized = false;
let followButton = null;

// Initialize the GPS map
function initMap() {
    // Create the map if it doesn't exist
    if (!map) {
        map = L.map('gps-map').setView([40.785091, -73.968285], 18);
        
        // Add OpenStreetMap tiles
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
            maxZoom: 19,
        }).addTo(map);
        
        // Create robot marker with a more visible icon
        const robotIcon = L.divIcon({
            className: 'robot-marker',
            html: '<div style="font-size: 24px; background-color: white; border-radius: 50%; width: 30px; height: 30px; text-align: center; line-height: 30px; border: 2px solid blue;">🤖</div>',
            iconSize: [30, 30],
            iconAnchor: [15, 15]
        });
        
        robotMarker = L.marker([40.785091, -73.968285], { icon: robotIcon }).addTo(map);
        
        // Add a click handler to toggle follow mode - stop following when map is clicked
        map.on('click', function() {
            setFollowMode(false);
        });
        
        // Add a button to toggle follow mode
        followButton = L.control({ position: 'topright' });
        followButton.onAdd = function() {
            const div = L.DomUtil.create('div', 'follow-button');
            div.innerHTML = '<button id="follow-toggle-btn" style="padding: 5px; cursor: pointer; background-color: #4CAF50; color: white; border: none; border-radius: 3px;">Following</button>';
            
            // Prevent the click event from propagating to the map
            L.DomEvent.disableClickPropagation(div);
            
            // Add click handler to toggle follow mode
            div.querySelector('#follow-toggle-btn').addEventListener('click', function(e) {
                // Toggle follow mode
                setFollowMode(!followRobot);
            });
            return div;
        };
        followButton.addTo(map);
        
        // Mark as initialized
        mapInitialized = true;
    }
}

// Set follow mode with visual feedback
function setFollowMode(follow) {
    followRobot = follow;
    
    // Update button text and style
    const followBtn = document.querySelector('#follow-toggle-btn');
    if (followBtn) {
        if (follow) {
            followBtn.textContent = 'Following';
            followBtn.style.backgroundColor = '#4CAF50'; // Green
            console.log("Follow mode enabled");
        } else {
            followBtn.textContent = 'Follow Robot';
            followBtn.style.backgroundColor = '#2196F3'; // Blue
            console.log("Follow mode disabled");
        }
    }
    
    // If enabling follow, update map position immediately
    if (follow) {
        updateGpsMap();
    }
}

// Update the GPS map with new data
function updateGpsMap() {
    fetch('/gps_data')
        .then(response => response.json())
        .then(data => {
            // Only proceed if we have valid latitude and longitude
            if (data.latitude && data.longitude) {
                const lat = data.latitude;
                const lng = data.longitude;
                const pos = [lat, lng];
                
                // Update position text
                document.getElementById('gps-position').textContent = `${lat.toFixed(6)}, ${lng.toFixed(6)}`;
                
                // Update geofence status
                const geofenceStatusEl = document.getElementById('geofence-status');
                geofenceStatusEl.textContent = data.geofence.inside ? 'Inside' : 'OUTSIDE';
                geofenceStatusEl.className = data.geofence.inside ? 'inside' : 'outside';
                
                // Display distance to edge
                const distanceToEdgeEl = document.getElementById('distance-to-edge');
                distanceToEdgeEl.textContent = data.geofence.distance_to_edge.toFixed(2);
                
                // Update satellite info
                document.getElementById('satellites-count').textContent = data.satellites || 0;
                document.getElementById('fix-quality').textContent = data.fix_quality || 0;
                
                // Initialize map if needed
                if (!mapInitialized) {
                    initMap();
                }
                
                // Always update the marker position
                robotMarker.setLatLng(pos);
                
                // Add or update the geofence circle
                if (!geofenceCircle && data.geofence_config) {
                    const config = data.geofence_config;
                    geofenceCircle = L.circle(
                        [config.center_lat, config.center_lon], 
                        {
                            radius: config.radius,
                            color: 'red',
                            fillColor: '#f03',
                            fillOpacity: 0.1,
                            weight: 2,
                            dashArray: '5, 5'
                        }
                    ).addTo(map);
                    
                    // Also add a warning circle
                    const warningRadius = config.radius - config.warning_distance;
                    if (warningRadius > 0) {
                        L.circle(
                            [config.center_lat, config.center_lon],
                            {
                                radius: warningRadius,
                                color: 'orange',
                                fillColor: '#ff6500',
                                fillOpacity: 0.05,
                                weight: 1,
                                dashArray: '3, 5'
                            }
                        ).addTo(map);
                    }
                }
                
                // Center map on robot position if following is enabled
                if (followRobot) {
                    console.log("Following robot at", lat, lng);
                    map.setView(pos, map.getZoom());
                }
            }
        })
        .catch(error => console.error('Error fetching GPS data:', error));
}

// Set up human position controls
document.getElementById('x-position').addEventListener('input', function() {
    const xPosition = this.value;
    const distance = document.getElementById('distance').value;
    document.getElementById('x-value').textContent = xPosition;
    
    // Throttle the updates
    clearTimeout(updateTimeout);
    updateTimeout = setTimeout(() => {
        updateHumanPosition(xPosition, distance);
    }, THROTTLE_DELAY);
});

document.getElementById('distance').addEventListener('input', function() {
    const distance = this.value;
    const xPosition = document.getElementById('x-position').value;
    document.getElementById('distance-value').textContent = distance;
    
    // Throttle the updates
    clearTimeout(updateTimeout);
    updateTimeout = setTimeout(() => {
        updateHumanPosition(xPosition, distance);
    }, THROTTLE_DELAY);
});

// Create a function to handle the update
function updateHumanPosition(x, distance) {
    fetch(`/control/update_human_position?x=${x}&distance=${distance}`)
        .then(response => response.json())
        .catch(error => console.error('Error updating position:', error));
}

// Control button handlers
document.getElementById('startTracking').addEventListener('click', () => {
    fetch('/control/tracking')
        .then(response => response.json())
        .then(data => {
            currentModeSpan.textContent = `Mode: ${data.mode}`;
            statusDiv.innerHTML += `<br>Command sent: Start tracking`;
        });
});

document.getElementById('stopTracking').addEventListener('click', () => {
    fetch('/control/stop')
        .then(response => response.json())
        .then(data => {
            currentModeSpan.textContent = `Mode: ${data.mode}`;
            statusDiv.innerHTML += `<br>Command sent: Stop robot`;
        });
});

document.getElementById('startSearch').addEventListener('click', () => {
    fetch('/control/search')
        .then(response => response.json())
        .then(data => {
            currentModeSpan.textContent = `Mode: ${data.mode}`;
            statusDiv.innerHTML += `<br>Command sent: Start search`;
        });
});

// Create safety circle points
function generateSafetyCircle(safetyDistance) {
    const angles = [];
    const distances = [];
    for (let angle = 0; angle <= 360; angle += 5) {
        angles.push(angle);
        distances.push(safetyDistance);
    }
    return {angles, distances};
}

let safetyCircle = generateSafetyCircle(1)

// Create initial empty plot
Plotly.newPlot(plotDiv, [
    {
        type: 'scatterpolar',
        r: [],
        theta: [],
        mode: 'markers',
        name: 'Scan Points',
        marker: { size: 3, color: 'rgba(70, 130, 180, 0.5)' }
    },
    {
        type: 'scatterpolar',
        r: [],
        theta: [],
        mode: 'markers',
        name: 'Obstacles',
        marker: { size: 12, color: 'red', symbol: 'x' }
    },
    {
        type: 'scatterpolar',
        r: safetyCircle.distances,
        theta: safetyCircle.angles,
        mode: 'lines',
        name: 'Safety Zone',
        line: { color: 'rgba(255, 0, 0, 0.7)', width: 1, dash: 'dash' },
        hoverinfo: 'none'
    }
], {
    polar: {
        radialaxis: { range: [0, 5] },
        angularaxis: { direction: "clockwise", rotation: 90 }
    },
    showlegend: true
});

// Update plot with latest data
function updatePlot() {
    fetch('/data?_=' + new Date().getTime())
        .then(response => response.json())
        .then(data => {
            // Update status with more details
            console.log("Data received ", data.obstacles);
            statusDiv.innerHTML = `
                <strong>Cycle:</strong> ${data.cycle_count} | 
                <strong>Obstacles:</strong> ${data.obstacles.length} | 
                <strong>Points:</strong> ${data.distances.length}
            `;

            safetyCircle = generateSafetyCircle(data.safety_distance || 1.0);

            // Process obstacles
            const obstacleAngles = data.obstacles.map(obs => 
                (Math.atan2(obs.y, obs.x) * 180/Math.PI + 360) % 360
            );
            const obstacleDistances = data.obstacles.map(obs => 
                Math.sqrt(obs.x*obs.x + obs.y*obs.y)
            );
            
            // Create obstacle hover text with tracking info
            const hoverTexts = data.obstacles.map(obs => {
                const speed = obs.speed !== undefined ? obs.speed.toFixed(2) : 'N/A';
                return `ID: ${obs.id || 'N/A'}<br>` + 
                       `Distance: ${obs.distance.toFixed(2)}m<br>` +
                       `Speed: ${speed} m/s<br>` + 
                       `Position: (${obs.x.toFixed(2)}, ${obs.y.toFixed(2)})`;
            });
            
            // Create velocity vectors for obstacles
            const arrowTraces = [];
            data.obstacles.forEach(obs => {
                if (obs.vx !== undefined && obs.vy !== undefined && (obs.vx !== 0 || obs.vy !== 0)) {
                    // Calculate velocity vector
                    const r0 = Math.sqrt(obs.x*obs.x + obs.y*obs.y);
                    const theta0 = (Math.atan2(obs.y, obs.x) * 180/Math.PI + 360) % 360;
                    const scale = 1.0; 
                    const endX = obs.x + obs.vx * scale;
                    const endY = obs.y + obs.vy * scale;
                    const r1 = Math.sqrt(endX*endX + endY*endY);
                    const theta1 = (Math.atan2(endY, endX) * 180/Math.PI + 360) % 360;
                    
                    arrowTraces.push({
                        type: 'scatterpolar',
                        r: [r0, r1],
                        theta: [theta0, theta1],
                        mode: 'lines',
                        line: {
                            color: 'rgba(255, 165, 0, 0.8)',
                            width: 2
                        },
                        showlegend: false,
                        hoverinfo: 'none'
                    });
                }
            });

            // Determine obstacle marker colors based on speed
            const markerColors = data.obstacles.map(obs => {
                if (!obs.speed) return 'red';
                const maxSpeed = 1.0;
                const speedRatio = Math.min(obs.speed / maxSpeed, 1);
                const r = Math.round(255 * speedRatio);
                const g = Math.round(255 * (1 - speedRatio));
                return `rgb(${r}, ${g}, 0)`;
            });

            // Create plot data
            const plotData = [
                // Raw scan points
                {
                    type: 'scatterpolar',
                    r: data.distances,
                    theta: data.angles,
                    mode: 'markers',
                    name: 'Scan Points',
                    marker: { size: 3, color: 'rgba(70, 130, 180, 0.5)' }
                },
                // Obstacles with tracking data
                {
                    type: 'scatterpolar',
                    r: obstacleDistances,
                    theta: obstacleAngles,
                    mode: 'markers+text',
                    name: 'Tracked Obstacles',
                    text: data.obstacles.map(obs => obs.id),
                    textposition: 'top right',
                    textfont: {
                        family: 'sans-serif',
                        size: 12,
                        color: '#000000'
                    },
                    marker: { 
                        size: 12, 
                        color: markerColors,
                        symbol: 'circle' 
                    },
                    hovertext: hoverTexts,
                    hoverinfo: 'text'
                },
                // Safety zone
                {
                    type: 'scatterpolar',
                    r: safetyCircle.distances,
                    theta: safetyCircle.angles,
                    mode: 'lines',
                    name: 'Safety Zone',
                    line: { color: 'rgba(255, 0, 0, 0.7)', width: 1, dash: 'dash' },
                    hoverinfo: 'none'
                }
            ];

            // Add velocity vector arrows
            plotData.push(...arrowTraces);

            // Add human marker if available
            if (data.human) {
                const humanAngle = data.human.angle;
                const humanDistance = data.human.distance;
                
                // Human marker
                plotData.push({
                    type: 'scatterpolar',
                    r: [humanDistance],
                    theta: [humanAngle],
                    mode: 'markers+text',
                    name: 'Human',
                    text: ['👤'],
                    textposition: 'top center',
                    textfont: {
                        family: 'sans-serif',
                        size: 18,
                        color: '#000000'
                    },
                    marker: { 
                        size: 15, 
                        color: 'blue',
                        symbol: 'circle',
                        line: {
                            color: 'white',
                            width: 2
                        }
                    },
                    hoverinfo: 'text',
                    hovertext: `Human<br>Distance: ${humanDistance.toFixed(2)}m<br>Angle: ${humanAngle.toFixed(1)}°`,
                    showlegend: true
                });
                
                // Line from origin to human
                plotData.push({
                    type: 'scatterpolar',
                    r: [0, humanDistance],
                    theta: [0, humanAngle],
                    mode: 'lines',
                    line: {
                        color: 'rgba(0, 0, 255, 0.5)',
                        width: 2,
                        dash: 'dot'
                    },
                    showlegend: false,
                    hoverinfo: 'none'
                });
                
                // Distance ring around human
                const ringPoints = 36;
                const ringAngles = [];
                const ringDistances = [];
                
                for (let i = 0; i <= ringPoints; i++) {
                    const angle = (i / ringPoints) * 360;
                    ringAngles.push(angle);
                    ringDistances.push(data.human.distance);
                }
                
                plotData.push({
                    type: 'scatterpolar',
                    r: ringDistances,
                    theta: ringAngles,
                    mode: 'lines',
                    line: {
                        color: 'rgba(0, 0, 255, 0.3)',
                        width: 1
                    },
                    showlegend: false,
                    hoverinfo: 'none'
                });
            }

            // Update the plot
            Plotly.react(plotDiv, plotData, {
                polar: {
                    radialaxis: { range: [0, 5] },
                    angularaxis: { direction: "clockwise", rotation: 90 }
                },
                showlegend: true
            });
        })
        .catch(error => console.error('Error:', error));
}

// Initialize map when the page loads
document.addEventListener('DOMContentLoaded', initMap);

// Update every 250ms for LIDAR data
setInterval(updatePlot, 250);
updatePlot(); // Initial update

// Update every 1s for GPS data (GPS updates less frequently)
setInterval(updateGpsMap, 1000);
updateGpsMap(); // Initial update