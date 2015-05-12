:- module(action_properties, [
      action_property_restriction/3,      % ?Action, ?Property, ?Restriction
      action_property_some/3,             % ?Action, ?Property, ?Class
      action_property_some_distinct/3,    % ?Action, ?Property, ?Class
      action_property_has/3,              % ?Action, ?Property, ?Value
      action_property_has_distinct/3,     % ?Action, ?Property, ?Value
      action_property_value_satisfies/3,  % ?Action, ?Property, ?Value
      action_on_object/2                  % ?Action, ?Object
   ]).

:- use_module(library('semweb/rdf_db.pl')).
:- use_module(library('knowrob_owl')).

:- rdf_meta
  action_property_restriction(r, r, t),
  action_property_some(r, r, r),
  action_property_some_distinct(r, r, r),
  action_property_has(r, r, r),
  action_property_has_distinct(r, r, r),
  action_property_value_satisfies(r, r, r),
  action_on_object(r, r).

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
    action_property_some(Action, Property, Class),
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
    action_property_has(Action, Property, Value),
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
% Evaluates to true if a TBOX action description satisfies all
% knowrob:objectActedOn property restrictions for an object
%
% @param Action Action class with at least one knowrob:objectActedOn property
%   restriction
% @param Object Object acted on
%
action_on_object(Action, Object) :-
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
