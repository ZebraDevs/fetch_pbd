^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetch_pbd_interaction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.8 (2017-08-08)
------------------
* Update README and CMakeLists
* Do reachability check for ref_type PREVIOUS_TARGET
* Interface improvements: update markers more efficiently, etc
* Implemented pre-conditions for primitives
* Interface improvements: object labels, marker fixes
* Namespace topics and services, some marker fixes
* Install specific version of polymer-cli to not get issues when polymer changes
* Change World node to C++ and add Grasp suggestion integration
* Contributors: Sarah Elliott

0.0.7 (2017-04-20)
------------------
* Built frontend and now serve pre-built frontend directly
* add couchdb as run_depend, fix `#6 <https://github.com/fetchrobotics/fetch_pbd/issues/6>`_
* Custom ros3d.js to use unsubscribeTf and show mesh colours. Use local copies of Robot Web Tools libraries.
* Contributors: Sarah Elliott

0.0.6 (2016-10-29)
------------------
* Update install_web_dependencies.sh
* Contributors: Sarah Elliott

0.0.5 (2016-09-17)
------------------
* Update package.xml
* Contributors: Russell Toris

0.0.4 (2016-09-09)
------------------
* Dependencies again
* Contributors: Sarah Elliott

0.0.3 (2016-09-09)
------------------

0.0.2 (2016-09-09)
------------------
* Dependencies
* Contributors: Sarah Elliott

0.0.1 (2016-09-09)
------------------
* Update package.xml
* License and documentation updates
* Update examples
* Okay  last CMakelists.tx change...
* Fix some frame initialisation issue
* Accidentally deleted line before...
* Switch from GuiInput.msg to GuiInput.srv
* Less gross CMakelist/path stuff
* CMakelists change
* Loading/saving json files, still need to work on location of files
* Small fix for editing actions
* Small fix for deleting objects from world
* Require version of rail_segmentation
* Switched to use rail_segmentation
* Getting ready to be released
* Small fix for deleting actions
* Fix frame issues when reordering primitives
* Fix bug in reordering primitives
* Marker UI fixes
* Small fixes so things appear better on mobile
* Error message popups
* Add table to planning scene
* Add functionality for stopping execution
* Fixes based on Russell's comments
* Change dependencies and action selection
* Misc reorganisation and added to example script
* Fix for trajectory marker
* Coloring for selected primitive
* Changes to selecting a primitive and structure of markers functionality
* More marker fixes. Probably needs a lot of cleanup now.
* Really made the markers work this time...
* Marker and UI fixes
* Fixed the no sound/no head movement behaviour
* Lots of changes but added ability to edit actions numerically
* Documentation and code cleanup
* Make my own sortable list
* Fix object marker location for web viz
* Get rid of arrows pointing to objects
* Added markers being relative to each other
* Show and hide markers, plus some other marker fixes
* Polymer web interface
* Add back the standalone arm control node
* Huge reorganisation. Now working.
* Huge reorganisation. Now working.
* Make continuous trajectories work (although crappily)
* Fixed bug in moving to marker location
* Changes to make 'links' between steps work
* Found small naming bug in unused service
* Cleaning up code
* Replace moveit_python with moveit_commander and use poses instead of joints
* Changed ArmControls class to ArmInteraction, fixed bug in PbD
* Working, all functionality of PR2 PbD
* First commit of port of PbD to Fetch. Things work. Rough around the edges.
* Contributors: Russell Toris, Sarah Elliott
