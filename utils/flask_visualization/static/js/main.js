// Initialize plot and variables
const plotDiv = document.getElementById('plot');
const statusDiv = document.getElementById('status');
const currentModeSpan = document.getElementById('current-mode');

let updateTimeout = null;
const THROTTLE_DELAY = 100; // milliseconds

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
function generateSafetyCircle() {
    const angles = [];
    const distances = [];
    for (let angle = 0; angle <= 360; angle += 5) {
        angles.push(angle);
        distances.push(1);
    }
    return {angles, distances};
}

const safetyCircle = generateSafetyCircle();

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
            statusDiv.innerHTML = `
                <strong>Cycle:</strong> ${data.cycle_count} | 
                <strong>Obstacles:</strong> ${data.obstacles.length} | 
                <strong>Points:</strong> ${data.distances.length}
            `;

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

// Update every 250ms
setInterval(updatePlot, 250);
updatePlot(); // Initial update