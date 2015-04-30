%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies

:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_map_tools).
:- register_ros_package(knowrob_objects).
:- register_ros_package(knowrob_actions).
:- register_ros_package(knowrob_semantic_map_tools).

:- use_module(library('semantic_map')).
:- use_module(library('owl_string_parser')).
:- use_module(library('object_properties')).
:- use_module(library('data_properties')).
:- use_module(library('action_properties')).
:- use_module(library('marker_interaction')).
:- use_module(library('marker_visualization')).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% register namespace
:- rdf_db:rdf_register_prefix(semantic_map,
     'http://asl.ethz.ch/knowrob/semantic_map.owl#',
     [keep(true)]
   ).
