:- module(semantic_map, [
      map_object_info/2,                    % +Objects, -Info
      map_object_frame/2,                   % Object, -Frame
      map_object_stamp/2                    % Object, -Stamp
   ]).

:- use_module(library('semweb/rdf_db.pl')).
:- use_module(library('semweb/rdfs.pl')).
:- use_module(library('semantic_map_utils.pl')).
:- use_module(library('knowrob_objects.pl')).
:- use_module(library('comp_temporal.pl')).

:- rdf_meta
  map_object_frame(r, -),
  map_object_stamp(r, -).

%% map_object_info(+Objects, -Info)
%
% Construct a list of properties of a list of map objects.
% 
% @param Objects List of map objects for which to retrieve information
% @param Info List of properties of the map objects
% 
map_object_info([Object|Rest], Info) :-
  map_object_info_1(Object, Info);
  map_object_info(Rest, Info).

map_object_info_1(Object, [Identifier, Type, Label, Frame, Stamp, Pose,
    [Width, Height, Depth], Parent]) :-
  Identifier = Object,
  map_object_type(Object, Type),
  rdfs_label(Object, Label),
  (
    map_object_frame(Object, Frame) ->
      true;
      Frame = ''
  ),
  (
    map_object_stamp(Object, Stamp) ->
      true;
      Stamp = 0
  ),
  (
    current_object_pose(Object, Pose) ->
      true;
      Pose = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  ),
  (
    map_object_dimensions(Object, Width, Depth, Height) ->
      true;
      (
        Width = 0,
        Height = 0,
        Depth = 0
      )
  ),
  (
    map_child_object(Parent, Object) ->
      true;
      map_instance(Parent)
  ).

%% map_object_frame(+Object, -Frame)
%
% Get tf frame of a map object.
% 
% @param Object Object for which to retrieve the tf frame
% @param Frame Tf frame of the map objects
% 
map_object_frame(Object, Frame) :-
  rdf_triple(knowrob:tfFrame, Object, literal(type(_, Frame))), !;
  (
    map_child_object(Parent, Object) ->
      map_object_frame(Parent, Frame);
      (
        map_instance(Map),
        rdf_triple(knowrob:tfFrame, Map, literal(type(_, Frame)))
      )
  ).
  
%% map_object_stamp(+Object, -Stamp)
%
% Get timestamp of the latest detection of a map object.
% 
% @param Object Object for which to retrieve the detection timestamp
% @param Stamp Stamp value of the map object in [s]
%
:- use_module(library('knowrob_objects.pl'),
                      [latest_detection_of_instance/2]).

map_object_stamp(Object, Stamp) :-
  latest_detection_of_instance(Object, Detection),
  rdf_triple(knowrob:startTime, Detection, Time),
  time_point_value(Time, Stamp).
