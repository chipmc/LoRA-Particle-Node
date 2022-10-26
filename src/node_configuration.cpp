/**
 * @brief This function is called in setup to set the specific settings for each node
 * 
 */

#include "Particle.h"
#include "MyPersistentData.h"

void setNodeConfiguration() {
  Log.info("Setting values for the node");
  sysStatus.set_sensorType(true); // Default is the car counter (true for PIR)
  // sysStatus.set_deviceID(32148);
  // sysStatus.set_structuresVersion(14);
  // sysStatus.set_nodeNumber(11);

  // sysStatus.flush(true);
}