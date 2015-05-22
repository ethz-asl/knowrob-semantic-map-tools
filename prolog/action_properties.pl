:- module(action_properties, [
      action_property/3,                  % ?Action, ?Property, ?Value
      action_property_some/3,             % ?Action, ?Property, ?Class
      action_property_some_distinct/3,    % ?Action, ?Property, ?Class
      action_property_has/3,              % ?Action, ?Property, ?Value
      action_property_has_distinct/3,     % ?Action, ?Property, ?Value
      action_property_value_satisfies/3,  % ?Action, ?Property, ?Value
      action_on_object/2,                 % ?Action, ?Object
      action_on_object_satisfies/2,       % ?Action, ?Object
      action_subactions/2,                % +Action, ?Subactions
      action_involving_object/2           % ?Action, ?Object
   ]).

:- use_module(library('semweb/rdf_db.pl')).
:- use_module(library('knowrob_owl')).

:- rdf_meta
  action_property(r, r, r),
  action_property_restriction(r, r, t),
  action_property_some(r, r, r),
  action_property_some_distinct(r, r, r),
  action_property_has(r, r, r),
  action_property_has_distinct(r, r, r),
  action_property_value_satisfies(r, r, r),
  action_on_object(r, r),
  action_on_object_satisfies(r, r),
  action_subactions(r, -),
  action_involving_object(r, r).

%% action_property(?Action, ?Property, ?Value)
%
% Reads the property for an ABOX action description
%
% @param Action Action individual with property
% @param Property Action property
% @param Value Value of the property for this action
% 
action_property(Action, Property, Value) :-
  owl_individual_of(Action, knowrob:'Action'),
  owl_has(Action, Property, Value).
  
%% action_property_restriction(?Action, ?Property, ?Restriction)
%
% Reads the property restriction for a TBOX action description
%
% @param Action Action class with a restriction on property
% @param Property Action property
% @param Restriction Restriction on the property for this action
% 
action_property_restriction(Action, Property, Restriction) :-
  owl_subclass_of(Action, knowrob:'Action'),
  owl_direct_subclass_of(Action, Sup),
  not(rdf_is_bnode(Action)),
  (
    (
      owl_direct_subclass_of(Sup, Sup2),
      owl_restriction(Sup2, restriction(Property, Restriction))
    );
    owl_restriction(Sup, restriction(Property, Restriction))
  ).

%% action_property_some(?Action, ?Property, ?Class)
%
% Reads the class restriction property for a TBOX action description
%
% @param Action Action class with a class restriction on property
% @param Property Action property
% @param Class Class set as restricton on property for this action
% 
action_property_some(Action, Property, Class) :-
  action_property_restriction(Action, Property, some_values_from(Class)).

%% action_property_some_distinct(?Action, ?Property, ?Class)
%
% Reads the class restriction property for a TBOX action description
% with distinction, i.e., only delivers the most derived TBOX actions
%
% @param Action Action class with a class restriction on property
% @param Property Action property
% @param Class Class set as restricton on property for this action
% 
action_property_some_distinct(Action, Property, Class) :-
  setof(Action2,
    action_property_some(Action2, Property, Class),
    Actions
  ),  
  member(Action, Actions),
  findall(SubAction,
    (
      member(SubAction, Actions),
      not(Action == SubAction),
      owl_subclass_of(SubAction, Action)
    ),
    []
  ).

%% action_property_has(?Action, ?Property, ?Value)
%
% Reads the value restriction property for a TBOX action description
%
% @param Action Action class with a value restriction on property
% @param Property Action property
% @param Value Value set as restricton on property for this action
% 
action_property_has(Action, Property, Value) :-
  action_property_restriction(Action, Property, has_value(Value)).

%% action_property_has_distinct(?Action, ?Property, ?Value)
%
% Reads the value restriction property for a TBOX action description
% with distinction, i.e., only delivers the most derived TBOX actions
%
% @param Action Action class with a value restriction on property
% @param Property Action property
% @param Value Value set as restricton on property for this action
% 
action_property_has_distinct(Action, Property, Value) :-
  setof(Action2,
    action_property_has(Action2, Property, Value),
    Actions
  ),
  member(Action, Actions),
  findall(SubAction,
    (
      member(SubAction, Actions),
      not(Action == SubAction),
      owl_subclass_of(SubAction, Action)
    ),
    []
  ).

%% action_property_value_satisfies(?Action, ?Property, ?Value)
%
% True if all class and value restrictions on a TBOX action description
% property are satisfied at once by a given value
%
% @param Action Action class with at least one restriction on property
% @param Property Action property
% @param Value Value of the property for this action
% 
action_property_value_satisfies(Action, Property, Value) :-
  (
    action_property_restriction(Action, Property, has_value(_)) ->
      action_property_has(Action, Property, Value);
      true
  ),
  (
    action_property_restriction(Action, Property, some_values_from(_)) ->
    (
      findall(Class,
        action_property_restriction(Action, Property, some_values_from(Class)),
        Classes
      ),
      forall(
        member(Class, Classes),
        owl_individual_of(Value, Class)
      )
    );
    true
  ).

%% action_on_object(?Action, ?Object)
%
% Evaluates to true for an ABOX action whose knowrob:objectActedOn property
% refers to object
%
% @param Action Action individual with knowrob:objectActedOn property
% @param Object Object acted on
%
action_on_object(Action, Object) :-
  action_property(Action, knowrob:objectActedOn, Object).

%% action_on_object_satisfies(?Action, ?Object)
%
% Evaluates to true if a TBOX action description satisfies all
% knowrob:objectActedOn property restrictions for an object
%
% @param Action Action class with at least one knowrob:objectActedOn property
%   restriction
% @param Object Object acted on
%
action_on_object_satisfies(Action, Object) :-
  findall(Action2,
    (
      action_property_restriction(Action2, knowrob:objectActedOn, _),
      action_property_value_satisfies(Action2, knowrob:objectActedOn, Object)
    ),
    Actions2
  ),
  sort(Actions2, Actions),
  member(Action, Actions),
  findall(SubAction,
    (
      member(SubAction, Actions),
      not(Action == SubAction),
      owl_subclass_of(SubAction, Action)
    ),
    []
  ).

%% action_subactions
%
% Read all sub-actions of an action.
%
% @param Action Action identifier
% @param Subactions List of subaction identifiers
% 
action_subactions(Action, Subactions) :-
  plan_subevents(Action, Subactions).
  
%% action_involving_object
%
% Evaluates to true if action contains any subaction on object.
%
% @param Action Action containing any subaction on object
% @param Object Identifier of the object acted on
% 
action_involving_object(Action, Object) :-
  owl_subclass_of(Action, knowrob:'Action'),
  action_subactions(Action, Subactions),
  findall(Object2,
    (
      member(Subaction, Subactions),
      action_property_has(Subaction, knowrob:objectActedOn, Object2)
    ),
    Objects2
  ),
  sort(Objects2, Objects),
  member(Object, Objects).
