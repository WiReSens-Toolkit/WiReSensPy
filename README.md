# WiSensPy

Python library for wireless collection and visualization of resistive tactile sensing 

## Pre-Requisities

The following assumes you already have the following installed on your machine:

- Python 3.10 or higher ([Installation](https://www.python.org/downloads/))
- Node and NPM v20.14.0 or higher ([Installation](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm))

## Installation

1. **Clone the repository**:

   ```bash
   git clone https://github.com/WiReSens-Toolkit/WiReSensPy.git
   cd WiReSensPy
   ```

### Python Setup

1. **Create a virtual environment** (optional but recommended):

   - For Python 3.x:
     ```bash
     python3 -m venv venv
     source venv/bin/activate  # On Windows: venv\Scripts\activate
     ```

2. **Install the dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

### Node Setup
1. **Install node packages**
```bash
cd ui/nextjs-flask
npm i
```

## Getting Started

You will use a WiReSens Toolkit JSON configuration to configure your tactile sensor readout and visualization. In this section we discuss basic setup for reading and visualizing a 16x16 sensor array over BLE connection. For more specific details on all possible JSON configurations, view WiSensConfig.json.


