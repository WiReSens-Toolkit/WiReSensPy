// pages/index.js
"use client";
import React, { useEffect, useState, useRef } from "react";
import dynamic from "next/dynamic";
import { io } from "socket.io-client";
import InteractiveHeatmap from "./interactiveheatmap";
import Toolbar from "./toolbar";
import styles from "./page.module.css";
import WiSensConfig from "../../../WiSensConfigClean.json";

const socket = io("http://127.0.0.1:5328"); // Adjust the URL as necessary

// Function to generate a 10x10 array with random values
const generateRandomArray = (rows, cols) => {
  const array = [];
  for (let i = 0; i < cols; i++) {
    const row = [];
    for (let j = 0; j < rows; j++) {
      row.push(Math.random());
    }
    array.push(row);
  }
  return array;
};

const Home = () => {
  let defaultSensors = {};
  WiSensConfig.sensors.map((sensorConfig) => {
    defaultSensors[sensorConfig.id] = generateRandomArray(
      sensorConfig.numReadWires,
      sensorConfig.numGroundWires
    );
  });
  const [sensors, setSensors] = useState(defaultSensors);
  const [selectMode, setSelectMode] = useState(false);
  const [eraseMode, setEraseMode] = useState(false);

  const onSelectNodesClick = (event) => {
    setSelectMode(!selectMode);
  };

  const onEraseModeClick = (event) => {
    setEraseMode(!eraseMode);
  };

  const sensorDivRef = useRef(null);

  useEffect(() => {
    const handleSensorData = (data) => {
      let dataObj = JSON.parse(data);
      setSensors(dataObj);
    };

    socket.on("connect", () => {
      console.log("Connected to server");
    });

    socket.on("disconnect", () => {
      console.log("Disconnected from server");
    });

    socket.on("sensor_data", handleSensorData);

    return () => {
      socket.off("sensor_data", handleSensorData);
    };
  }, []);

  return (
    <div className={styles.pageDiv}>
      <Toolbar
        onSelectNodes={onSelectNodesClick}
        onRemoveNodes={onEraseModeClick}
        eraseMode={eraseMode}
        selectMode={selectMode}
      ></Toolbar>
      <div className={styles.sensorDiv}>
        {WiSensConfig.sensors.map((sensorId) => (
          <div key={sensorId.id} className={styles.heatmapContainer}>
            <div className={styles.sensorTitle}>{`Sensor ${sensorId.id}`}</div>
            <div className={styles.interactiveHeatmapDiv} ref={sensorDivRef}>
              <InteractiveHeatmap
                data={sensors[sensorId.id]}
                sensorDivRef={sensorDivRef}
                pitch={WiSensConfig.vizOptions.pitch}
                outlineImage={sensorId.outlineImage}
                selectMode={selectMode}
                eraseMode={eraseMode}
                setSelectMode={setSelectMode}
              ></InteractiveHeatmap>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};

export default Home;
