// Global variables
let map;
let locations = [];
let currentRoute = null;
let currentStepIndex = 0;
let routeSteps = [];
let routePolyline = null;
let startMarker = null;
let endMarker = null;
let recognition = null;

// Initialize the application
document.addEventListener('DOMContentLoaded', function() {
    initializeMap();
    loadLocations();
    setupEventListeners();
    setupVoiceRecognition();
});

// Initialize Leaflet map
function initializeMap() {
    map = L.map('map').setView([24.9325, 67.1139], 16);
    
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© OpenStreetMap contributors'
    }).addTo(map);
    
    const nedBounds = [
        [24.928, 67.105],
        [24.940, 67.120]
    ];
    
    L.rectangle(nedBounds, {
        color: '#3498db',
        weight: 2,
        fillOpacity: 0.1
    }).addTo(map);
}

// Load locations from server
async function loadLocations() {
    try {
        const response = await fetch('/api/locations');
        locations = await response.json();
        
        const datalist = document.getElementById('locationList');
        datalist.innerHTML = '';
        
        locations.forEach(location => {
            const option = document.createElement('option');
            option.value = location.name;
            datalist.appendChild(option);
            
            L.marker([location.lat, location.lon])
                .addTo(map)
                .bindPopup(`<h3>${location.name}</h3><p>Lat: ${location.lat.toFixed(6)}<br>Lon: ${location.lon.toFixed(6)}</p>`);
        });
        
        console.log(`Loaded ${locations.length} locations`);
    } catch (error) {
        console.error('Error loading locations:', error);
        showError('Failed to load locations');
    }
}

// Setup event listeners
function setupEventListeners() {
    document.getElementById('routeBtn').addEventListener('click', findRoute);
    document.getElementById('clearBtn').addEventListener('click', clearRoute);
    document.getElementById('voiceBtn').addEventListener('click', toggleVoiceInput);
    document.getElementById('speakStepBtn').addEventListener('click', speakCurrentStep);
    document.getElementById('nextStepBtn').addEventListener('click', nextStep);
    
    document.getElementById('startInput').addEventListener('keypress', function(e) {
        if (e.key === 'Enter') findRoute();
    });
    
    document.getElementById('destInput').addEventListener('keypress', function(e) {
        if (e.key === 'Enter') findRoute();
    });
}

// Setup voice recognition
function setupVoiceRecognition() {
    if ('webkitSpeechRecognition' in window || 'SpeechRecognition' in window) {
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        recognition = new SpeechRecognition();
        
        recognition.continuous = false;
        recognition.interimResults = false;
        recognition.lang = 'en-US';
        
        recognition.onstart = function() {
            document.getElementById('voiceStatus').textContent = 'Listening...';
            document.getElementById('voiceStatus').className = 'muted listening';
        };
        
        recognition.onresult = function(event) {
            const transcript = event.results[0][0].transcript.toLowerCase();
            document.getElementById('voiceStatus').textContent = 'Processing...';
            document.getElementById('voiceStatus').className = 'muted processing';
            
            processVoiceCommand(transcript);
        };
        
        recognition.onerror = function(event) {
            document.getElementById('voiceStatus').textContent = 'Voice recognition error';
            document.getElementById('voiceStatus').className = 'muted';
        };
        
        recognition.onend = function() {
            document.getElementById('voiceStatus').textContent = '';
            document.getElementById('voiceStatus').className = 'muted';
        };
    } else {
        document.getElementById('voiceBtn').disabled = true;
        document.getElementById('voiceStatus').textContent = 'Voice not supported';
    }
}

// Process voice commands
function processVoiceCommand(transcript) {
    console.log('Voice command:', transcript);
    
    const words = transcript.split(' ');
    let startLocation = '';
    let endLocation = '';
    
    const fromIndex = words.indexOf('from');
    const toIndex = words.indexOf('to');
    
    if (fromIndex !== -1 && toIndex !== -1 && toIndex > fromIndex) {
        startLocation = words.slice(fromIndex + 1, toIndex).join(' ');
        endLocation = words.slice(toIndex + 1).join(' ');
    } else if (words.includes('navigate') || words.includes('go')) {
        const navIndex = Math.max(words.indexOf('navigate'), words.indexOf('go'));
        if (navIndex !== -1) {
            endLocation = words.slice(navIndex + 2).join(' ');
        }
    }
    
    if (startLocation) {
        const matchedStart = findBestLocationMatch(startLocation);
        if (matchedStart) {
            document.getElementById('startInput').value = matchedStart.name;
        }
    }
    
    if (endLocation) {
        const matchedEnd = findBestLocationMatch(endLocation);
        if (matchedEnd) {
            document.getElementById('destInput').value = matchedEnd.name;
        }
    }
    
    if (document.getElementById('startInput').value && document.getElementById('destInput').value) {
        setTimeout(findRoute, 500);
    }
    
    document.getElementById('voiceStatus').textContent = 'Voice command processed';
}

// Find best matching location
function findBestLocationMatch(query) {
    query = query.toLowerCase();
    let bestMatch = null;
    let bestScore = 0;
    
    locations.forEach(location => {
        const name = location.name.toLowerCase();
        let score = 0;
        
        if (name === query) {
            score = 100;
        } else if (name.includes(query)) {
            score = 80;
        } else if (query.includes(name)) {
            score = 60;
        } else {
            const queryWords = query.split(' ');
            const nameWords = name.split(' ');
            let wordMatches = 0;
            
            queryWords.forEach(qWord => {
                nameWords.forEach(nWord => {
                    if (qWord === nWord || qWord.includes(nWord) || nWord.includes(qWord)) {
                        wordMatches++;
                    }
                });
            });
            
            score = (wordMatches / Math.max(queryWords.length, nameWords.length)) * 40;
        }
        
        if (score > bestScore) {
            bestScore = score;
            bestMatch = location;
        }
    });
    
    return bestScore > 30 ? bestMatch : null;
}

// Toggle voice input
function toggleVoiceInput() {
    if (recognition) {
        recognition.start();
    }
}

// Find route between locations
async function findRoute() {
    const startName = document.getElementById('startInput').value.trim();
    const endName = document.getElementById('destInput').value.trim();
    
    if (!startName || !endName) {
        showError('Please enter both start and destination locations');
        return;
    }
    
    if (startName === endName) {
        showError('Start and destination cannot be the same');
        return;
    }
    
    const routeBtn = document.getElementById('routeBtn');
    const originalText = routeBtn.textContent;
    routeBtn.innerHTML = '<span class="loading"></span> Finding Route...';
    routeBtn.disabled = true;
    
    try {
        const response = await fetch('/api/route', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                start: startName,
                end: endName
            })
        });
        
        const result = await response.json();
        
        if (result.success) {
            displayRoute(result);
            speak(`Route found! Total distance: ${result.total_distance} meters`);
        } else {
            showError(result.error || 'No route found');
        }
    } catch (error) {
        console.error('Error finding route:', error);
        showError('Failed to find route');
    } finally {
        routeBtn.textContent = originalText;
        routeBtn.disabled = false;
    }
}

// Display route on map and in steps
function displayRoute(routeData) {
    clearRoute();
    
    currentRoute = routeData;
    routeSteps = routeData.steps;
    currentStepIndex = 0;
    
    const routeCoords = routeData.path.map(point => [point.lat, point.lon]);
    routePolyline = L.polyline(routeCoords, {
        color: '#e74c3c',
        weight: 4,
        opacity: 0.8
    }).addTo(map);
    
    const startPoint = routeData.path[0];
    const endPoint = routeData.path[routeData.path.length - 1];
    
    startMarker = L.marker([startPoint.lat, startPoint.lon], {
        icon: L.icon({
            iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-green.png',
            shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
            iconSize: [25, 41],
            iconAnchor: [12, 41],
            popupAnchor: [1, -34],
            shadowSize: [41, 41]
        })
    }).addTo(map).bindPopup(`<h3>Start: ${startPoint.name}</h3>`);
    
    endMarker = L.marker([endPoint.lat, endPoint.lon], {
        icon: L.icon({
            iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-red.png',
            shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
            iconSize: [25, 41],
            iconAnchor: [12, 41],
            popupAnchor: [1, -34],
            shadowSize: [41, 41]
        })
    }).addTo(map).bindPopup(`<h3>End: ${endPoint.name}</h3>`);
    
    map.fitBounds(routePolyline.getBounds(), { padding: [20, 20] });
    
    displaySteps();
}

// Display navigation steps
function displaySteps() {
    const stepsBox = document.getElementById('stepsBox');
    stepsBox.innerHTML = '';
    
    if (routeSteps.length === 0) {
        stepsBox.innerHTML = '<p class="muted">No steps available</p>';
        return;
    }
    
    routeSteps.forEach((step, index) => {
        const stepDiv = document.createElement('div');
        stepDiv.className = `step ${index === currentStepIndex ? 'current' : ''}`;
        stepDiv.innerHTML = `
            <div class="step-instruction">${step.instruction}</div>
            <div class="step-distance">${step.distance}m</div>
        `;
        stepsBox.appendChild(stepDiv);
    });
    
    document.getElementById('speakStepBtn').disabled = false;
    document.getElementById('nextStepBtn').disabled = false;
}

// Speak current step
function speakCurrentStep() {
    if (routeSteps.length > 0 && currentStepIndex < routeSteps.length) {
        const step = routeSteps[currentStepIndex];
        speak(`Step ${currentStepIndex + 1}: ${step.instruction}. Distance: ${step.distance} meters.`);
    }
}

// Move to next step
function nextStep() {
    if (currentStepIndex < routeSteps.length - 1) {
        currentStepIndex++;
        displaySteps();
        speakCurrentStep();
    } else {
        speak('You have reached your destination!');
    }
}

// Clear current route
function clearRoute() {
    if (routePolyline) {
        map.removeLayer(routePolyline);
        routePolyline = null;
    }
    
    if (startMarker) {
        map.removeLayer(startMarker);
        startMarker = null;
    }
    
    if (endMarker) {
        map.removeLayer(endMarker);
        endMarker = null;
    }
    
    document.getElementById('startInput').value = '';
    document.getElementById('destInput').value = '';
    document.getElementById('stepsBox').innerHTML = '';
    
    currentRoute = null;
    routeSteps = [];
    currentStepIndex = 0;
    
    document.getElementById('speakStepBtn').disabled = true;
    document.getElementById('nextStepBtn').disabled = true;
    
    map.setView([24.9325, 67.1139], 16);
}

// Text-to-speech function
function speak(text) {
    if ('speechSynthesis' in window) {
        const utterance = new SpeechSynthesisUtterance(text);
        utterance.rate = 0.8;
        utterance.pitch = 1;
        utterance.volume = 1;
        speechSynthesis.speak(utterance);
    } else {
        console.log('Speech synthesis not supported:', text);
    }
}

// Show error message
function showError(message) {
    const stepsBox = document.getElementById('stepsBox');
    stepsBox.innerHTML = `<div style="color: #e74c3c; font-weight: bold; padding: 10px; background: #fff5f5; border-radius: 5px;">${message}</div>`;
    speak(message);
}