
set(PandarGeneralRaw ${CMAKE_CURRENT_SOURCE_DIR}/WingtechLidar_General_SDK/src/PandarGeneralRaw)

set(wingtechplugin_xml
  ${CMAKE_CURRENT_SOURCE_DIR}/WingtechPacketInterpreter/WingtechPacketInterpreter.xml
  ${CMAKE_CURRENT_SOURCE_DIR}/WingtechGeneralPacketInterpreter/WingtechGeneralPacketInterpreter.xml
  )

set(wingtechplugin_classes
  ${CMAKE_CURRENT_SOURCE_DIR}/WingtechPacketInterpreter/vtkWingtechPacketInterpreter
  ${CMAKE_CURRENT_SOURCE_DIR}/WingtechGeneralPacketInterpreter/vtkWingtechGeneralPacketInterpreter
  )

set(wingtechplugin_sources
  ${PandarGeneralRaw}/src/pandarGeneral_internal.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/WingtechPacketInterpreter/laser_ts.cxx
  )

#Create Module
vtk_module_add_module(WingtechLidar
  FORCE_STATIC # Using FORCE_STATIC build the vtk module statically into the plugin library, to avoid confusion when loading  
  SOURCES ${wingtechplugin_sources} # Will not trigger a PV wrapping
  CLASSES ${wingtechplugin_classes} # Those will be PV ClientServer wrapped #shorcut for sources x.cxx x.h  
  HEADERS ${wingtechplugin_headers} # Those will be PV ClientServer wrapped
)
paraview_add_server_manager_xmls(
  MODULE WingtechLidar
  XMLS  ${wingtechplugin_xml}
)

target_include_directories(WingtechLidar
  PUBLIC
  WingtechPacketInterpreter/
  WingtechGeneralPacketInterpreter
  ${PandarGeneralRaw}/include/
  ${PandarGeneralRaw}/include/pandarGeneral
  )

set(calib_files
  WT150.csv
)

# Copy the calibration to the build folder
foreach(file ${calib_files})
  configure_file("CalibrationFiles/${file}" "${PROJECT_BINARY_DIR}/share/${file}" COPYONLY)
endforeach()

# Install step
set(installfile_dest share)
if(APPLE)
  set(installfile_dest "${LV_INSTALL_LIBRARY_DIR}/../Resources")
endif()
foreach(file ${calib_files})
  install(FILES "${PROJECT_BINARY_DIR}/share/${file}" DESTINATION ${installfile_dest})
endforeach()
