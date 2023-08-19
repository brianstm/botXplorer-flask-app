# botXplorer AMR Flask App
[![License](https://img.shields.io/badge/License-Apache%202.0-red.svg)](https://opensource.org/licenses/Apache-2.0)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-Brians%20Tjipto-blue.svg)](https://www.linkedin.com/in/brians-tjipto-a25850153/)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-Aziel%20Chen-blue.svg)](https://www.linkedin.com/in/aziel-chen-a79594278/)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-Sameer%20Ahamed-blue.svg)](https://www.linkedin.com/in/sameer-ahamed-648543204)

## Description
This repository contains the Flask app package for the botXplorer Autonomous Mobile Robot (AMR) developed by Singapore Polytechnic and SICK Pte. Ltd Singapore. The botXplorer AMR is a revamped and remade version of the Ubiquity Silver Magni Bot, equipped with the SICK TiM781 LiDAR Sensor. It utilizes the RosPy ROS 1 Noetic framework for communication and control.

## Installation

Follow these steps to set up the botXplorer AMR Flask app:

1. Clone the repository:

   ```bash
   git clone https://github.com/your-username/botxplorer-amr-flask.git
   ```

2. Install the required dependencies:

   ```bash
   rosdep install package.xml
   ```

3. Run the Flask app:

   ```bash
   python app.py
   ```

4. Open the web browser and navigate to `http://localhost:6969` to access the botXplorer AMR interface.

## Documentation

Comprehensive documentation for the botXplorer AMR can be found at [https://amr-docs-brianstm.vercel.app](https://amr-docs-brianstm.vercel.app). It provides detailed information about the project, including setup instructions, API documentation, and usage guidelines.

## Navigation

For the navigation capabilities of the botXplorer AMR, please refer to the [botXplorer Navigation](https://github.com/brianstm/botXplorer-navigation) repository. It contains the code and documentation specifically related to the autonomous navigation functionality of the AMR.

## Features

- Integrated control interface to interact with the botXplorer AMR.
- Creation and execution of predefined routes using the botXplorer AMR's autonomous navigation capabilities.
- Monitoring of battery state and status messages.
- Confidence level estimation calculations powered by NumPy based on the AMR's localization covariance data.
- Particle cloud correctness estimation calculations powered by NumPy for accurate localization status.

## Contributing

Contributions to the development of the botXplorer AMR Flask app are welcome. If you encounter any issues or have suggestions for improvements, feel free to open an issue or submit a pull request on GitHub.

## Authors

- Brians Tjipto Meidianto
- Aziel Chen Qing Feng
- Sameer Ahamed Sadiq Batcha

## License

The botXplorer AMR Flask app is licensed under the [Apache 2.0 License](LICENSE).
