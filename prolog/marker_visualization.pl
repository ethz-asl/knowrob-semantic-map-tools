:- module(marker_visualization, [
      visualization_find_children/2,          % +Parents, -Children
      visualization_object_info/2             % +Objects, -Info
   ]).

:- use_module(library('semweb/rdf_db.pl')).
:- use_module(library('semweb/rdfs.pl')).
:- use_module(library('semantic_map_utils.pl')).
:- use_module(library('knowrob_objects.pl')).
:- use_module(library('knowrob_cad_parser.pl')).

%% visualization_find_children(+Parents, -Children)
%
% Find all children of a list of parent objects and construct a list
% which includes all parents as well as all children.
% 
% @param Parents List of parent objects
% @param Children List of parent and their child objects
% 
visualization_find_children([Parent|Rest], Children) :-
  findall(Child,
    (
      visualization_find_children_1(Parent, Child);
      visualization_find_children(Rest, Child)
    ),
    ChildrenWithDuplicates
  ),
  sort(ChildrenWithDuplicates, Children).

visualization_find_children_1(Parent, Child) :-
  rdf_reachable(DescribedInMap, knowrob:describedInMap, Parent),
  rdf_reachable(DescribedInMap, knowrob:properPhysicalParts, Child).

%% visualization_object_info(+Objects, -Info)
%
% Construct a list of properties required to visualize map objects.
% 
% @param Objects List of objects for which to retrieve information
% @param Info List of properties required to visualize the objects
% 
visualization_object_info([Object|Rest], Info) :-
  visualization_object_info_1(Object, Info);
  visualization_object_info(Rest, Info).

visualization_object_info_1(Object, [Identifier, Type, Label, Pose,
    [Width, Height, Depth], ModelPath]) :-
  map_object_info([Object], [Identifier, Type, Label, _, _, Pose,
    [Width, Height, Depth], _]),
  (
    get_model_path(Object, ModelPath) ->
      true;
      ModelPath = ''
  ).
