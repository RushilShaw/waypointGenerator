# GPS Simulation Commands Generator

This Python script generates GPS simulation commands based on a configuration file. It utilizes the Mapbox API and Open-Meteo API to obtain route information, coordinates, and elevations. The generated commands can be used for simulating GPS motion for the Jackson Labs GPS.

# Warning 

TODO INSERT THE WARNING

## Prerequisites

Before running the script, make sure you have the following:

- Python 3.x installed on your system
- Internet connection to access the Mapbox API and Open-Meteo API
- API key for Mapbox (required to access the API)
- Configuration file in JSON format (e.g., `config.json`)

## Installation

1. Clone the repository or download the script file (`gps_simulation_commands.py`).
2. Install the required Python packages by running the following command in your terminal:

   ```shell
   pip install geopy requests simplekml
   ```

## Usage

1. Update the `MAPBOX_API_KEY` constant in the script with your Mapbox API key.
2. Create a configuration file (`config.json`) with the following structure:

   ```json
   {
     "Starting Point": {
       "latitude": 37.7749,
       "longitude": -122.4194
     },
     "Stops": [
       {
         "latitude": 37.7815,
         "longitude": -122.3937
       },
       {
         "latitude": 37.7698,
         "longitude": -122.4862
       }
     ],
     "Ending Point": {
       "latitude": 37.7749,
       "longitude": -122.4194
     },
     "Output Filename": "commands.txt"
   }
   ```

   - `Starting Point`: The starting point coordinates (latitude and longitude) of the GPS simulation.
   - `Stops` (optional): An array of intermediate stop coordinates (latitude and longitude) for the GPS simulation.
   - `Ending Point`: The ending point coordinates (latitude and longitude) of the GPS simulation.
   - `Output Filename`: The name of the file to save the generated commands. (Use `.txt` extension for plain text format or `.kml` for KML format). If null is provided then the generated commands will be ouput in the console.

3. Run the script using the following command:

   ```shell
   python gps_simulation_commands.py
   ```

   The script will generate the GPS simulation commands based on the configuration file and save them to the specified output file.

4. After running the script, you will have a file with the generated GPS simulation commands (either plain text or KML format).

   **Plain Text Format:**
   The commands can be directly used in a simulator or GPS simulation system.

   **KML Format:**
   The KML file can be opened in mapping software (e.g., Google Earth) to visualize the generated waypoints and their corresponding altitudes and headings.

## Customization

You can customize the script according to your specific requirements:

- Modify the `MOST_ALLOWED_COORDINATES` constant to limit the maximum number of allowed coordinates in the generated commands.
- Adjust the `LATERAL_ACCELERATION` constant to change the lateral acceleration value for the generated commands.
- Customize the `CONFIGURATION_FILEPATH` constant to modify the generated output format or add additional output options.

## License

This script is released under the [MIT License](LICENSE).

## Disclaimer

This script relies on external APIs and services (Mapbox API and Open-Meteo API) to obtain route information, coordinates, and elevations. Please make sure you comply with the terms and conditions of these services while using this script.

