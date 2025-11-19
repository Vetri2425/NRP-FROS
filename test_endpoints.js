// test_endpoints.js
const API_BASE = 'http://192.168.1.101:5001';

async function testEndpoint(method, endpoint, body = null) {
  try {
    const url = `${API_BASE}${endpoint}`;
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), 5000); // 5 second timeout

    const options = {
      method,
      headers: { 'Content-Type': 'application/json' },
      signal: controller.signal,
    };
    if (body) options.body = JSON.stringify(body);

    const response = await fetch(url, options);
    clearTimeout(timeoutId);
    console.log(`${method} ${endpoint}: ${response.status} ${response.statusText}`);
    if (response.ok) {
      const data = await response.json();
      console.log('Response:', data);
    } else {
      console.log('Error response:', await response.text());
    }
  } catch (error) {
    if (error.name === 'AbortError') {
      console.log(`${method} ${endpoint}: Timeout - Server not responding`);
    } else {
      console.log(`${method} ${endpoint}: Failed - ${error.message}`);
    }
  }
}

async function runTests() {
  console.log('Testing Mission Control Endpoints...\n');

  // Test GET endpoints
  await testEndpoint('GET', '/api/mission/download');
  await testEndpoint('GET', '/api/mission/load_controller'); // New endpoint to check

  // Test POST endpoints (without actual data to avoid side effects)
  console.log('\nTesting POST endpoints (dry run - check if accepted):');

  // For upload, use dummy data
  await testEndpoint('POST', '/api/mission/upload', {
    waypoints: [{ id: 1, lat: 12.345, lng: 77.123, alt: 10, command: 'WAYPOINT' }]
  });

  // Pause/Resume/Clear (these might require active mission)
  await testEndpoint('POST', '/api/mission/pause');
  await testEndpoint('POST', '/api/mission/resume');
  await testEndpoint('POST', '/api/mission/clear');

  // Set current
  await testEndpoint('POST', '/api/mission/set_current', { wp_seq: 0 });

  // Set mode
  await testEndpoint('POST', '/api/set_mode', { mode: 'HOLD' });

  // Arm
  await testEndpoint('POST', '/api/arm', { value: false });

  console.log('\nEndpoint testing complete.');
}

runTests();