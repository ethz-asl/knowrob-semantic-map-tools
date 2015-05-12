:- module(marker_interaction, [
      get_handle_path/2,                    % +Identifier, -Path
      get_menu_title/2,                     % +Identifier, -Title
      interaction_find_children/2,          % +Parents, -Children
      interaction_object_info/2             % +Objects, -Info
   ]).

:- use_module(library('semweb/rdf_db.pl')).
:- use_module(library('semweb/rdfs.pl')).
:- use_module(library('semantic_map_utils.pl')).
:- use_module(library('knowrob_objects.pl')).
:- use_module(library('knowrob_cad_parser.pl')).
:- use_module(library('action_properties.pl')).
:- use_module(library('data_properties.pl')).

:- rdf_meta
  get_handle_path(r, ?),
  get_menu_title(r, ?).

:- rdf_db:rdf_register_prefix(marker_interaction,
     'http://asl.ethz.ch/knowrob/marker_interaction.owl#',
     [keep(true)]
   ).

%% get_handle_path(+Identifier, -Path)
%
% Searches for marker_interaction:pathToInteractiveMarkerHandle property
%
% @param Identifer Object instance or class identifier
% @param Path Found handle path
%
get_handle_path(Identifier, Path) :-
  data_property(Identifier,
    marker_interaction:pathToInteractiveMarkerHandle,
    Path
  ).

%% get_menu_title(?Identifier, ?Title)
%
% Searches for marker_interaction:menuEntryTitle property
% 
% @param Identifier Action instance or class identifier
% @param Title Found menu title
% 
get_menu_title(Identifier, Title) :-
  data_property(Identifier,
    marker_interaction:menuEntryTitle,
    Title
  ).

%% interaction_find_children(+Parents, -Children)
%
% Find all children of a list of parent objects and construct a list
% which includes all parents as well as all children.
% 
% @param Parents List of parent objects
% @param Children List of parent and their child objects
% 
interaction_find_children([Parent|Rest], Children) :-
  findall(Child,
    (
      interaction_find_children_1(Parent, Child);
      interaction_find_children(Rest, Child)
    ),
    ChildrenWithDuplicates
  ),
  sort(ChildrenWithDuplicates, Children).

interaction_find_children_1(Parent, Child) :-
  rdf_reachable(DescribedInMap, knowrob:describedInMap, Parent),
  rdf_reachable(DescribedInMap, knowrob:properPhysicalParts, Child).

%% interaction_object_info(+Objects, -Info)
%
% Construct a list of properties required to interact with map objects.
% 
% @param Objects List of objects for which to retrieve information
% @param Info List of properties required to interact with the objects
% 
interaction_object_info([Object|Rest], Info) :-
  interaction_object_info_1(Object, Info);
  interaction_object_info(Rest, Info).

interaction_object_info_1(Object, [Identifier, Type, Label, Pose,
    HandlePath, Actions]) :-
  Identifier = Object,
  map_object_type(Object, Type),
  rdfs_label(Object, Label),
  (
    current_object_pose(Object, _Pose) ->
      Pose = _Pose;
      Pose = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  ),
  (
    get_handle_path(Object, _HandlePath) ->
      HandlePath = _HandlePath;
      HandlePath = ''
  ),
  findall([ActionIdentifier, ActionTitle],
    (
      action_on_object(ActionIdentifier, Object),
      (
        get_menu_title(ActionIdentifier, _ActionTitle) ->
          ActionTitle = _ActionTitle;
          ActionTitle = ''
      )
    ),
    Actions
  ).
