:- module(semantic_map, [
      map_object_info/2                     % +Objects, -Info
   ]).

:- use_module(library('semweb/rdf_db.pl')).
:- use_module(library('semweb/rdfs.pl')).
:- use_module(library('semantic_map_utils.pl')).

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

map_object_info_1(Object, [Identifier, Type, Label, Pose,
    [Width, Height, Depth], Parent]) :-
  Identifier = Object,
  map_object_type(Object, Type),
  rdfs_label(Object, Label),
  (
    current_object_pose(Object, _Pose) ->
      Pose = _Pose;
      Pose = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  ),
  (
    map_object_dimensions(Object, _Width, _Depth, _Height) ->
      (
        Width = _Width,
        Height = _Height,
        Depth = _Depth
      );
      (
        Width = 0,
        Height = 0,
        Depth = 0
      )
  ),
  (
    map_child_object(_Parent, Object) ->
      Parent = _Parent;
      map_instance(Parent)
  ).
