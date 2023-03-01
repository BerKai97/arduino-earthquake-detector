  <h1>Earthquake Detection System using MPU6050</h1>
  <p>This is a robust and highly accurate earthquake detection system using the MPU6050 accelerometer and gyroscope. The system is designed to alert the user with a buzzer when an earthquake is detected.</p>
  <p>The system incorporates a custom smoothing algorithm to prevent false alarms, especially when the user is walking or when the system is on a moving vehicle like a car, bus, or train. The gyroscope is highly sensitive to movement, and the system takes this into account.</p>
  <h2>Components</h2>
  <p>The following components are required to build the system:</p>
  <table>
    <thead>
      <tr>
        <th>Component</th>
        <th>Quantity</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td>MPU6050 accelerometer and gyroscope</td>
        <td>1</td>
      </tr>
      <tr>
        <td>Potantiometer (I used 10k)</td>
        <td>1</td>
      </td>
      <tr>
        <td>Buzzer</td>
        <td>1</td>
      </tr>
      <tr>
        <td>RGB LED</td>
        <td>1</td>
      </tr>
      <tr>
        <td>Arduino Nano or equivalent</td>
        <td>1</td>
      </tr>
    </tbody>
  </table>
  <h2>Setup</h2>
  <p>To set up the system, follow these steps:</p>
  <ol>
    <li>Make the connections are as follows:</li>
  </ol>
  <table>
    <thead>
      <tr>
        <th>MPU6050</th>
        <th>Arduino</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td>VCC</td>
        <td>5V</td>
      </tr>
      <tr>
        <td>GND</td>
        <td>GND</td>
      </tr>
      <tr>
        <td>SDA</td>
        <td>A4</td>
      </tr>
      <tr>
        <td>SCL</td>
        <td>A5</td>
      </tr>
    </tbody>
    <thead>
        <tr>
            <th>LED</th>
            <th>Arduino</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td>Red</td>
            <td>D3</td>
        </tr>
        <tr>
            <td>Green</td>
            <td>D4</td>
        </tr>
        <tr>
            <td>Blue</td>
            <td>D5</td>
        </tr>
    </tbody>
   
  </table>
  
  <ol start="2">
    <li>Connect the buzzer to pin 12 on the Arduino and the Potantiometer to A3.</li>
  </ol>
  <h2>Usage</h2>
  <p>To use the system, turn on the Arduino and wait for the system to calibrate. The LED will turn on during calibration, which takes a few seconds. Once calibration is complete, the LED will turn off.</p>
  <p>The system is designed to detect earthquakes, and the buzzer will sound when an earthquake is detected. If the buzzer sounds, take appropriate action to protect yourself and your surroundings.</p>
    <h2>Contributing</h2>
        <p>Contributions are welcome. To contribute, fork the repository and create a new branch for your changes. Once your changes are complete, create a pull request to merge your changes back into the main branch.</p>
        <p>Please ensure that your changes are well-documented and that any new features or changes are thoroughly tested.</p>

