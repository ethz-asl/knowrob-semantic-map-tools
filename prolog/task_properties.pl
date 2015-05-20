:- module(task_properties, [
      task_actions/2,                           % +Task, ?Actions
      task_involving_object/2                   % ?Task, ?Object
   ]).

:- use_module(library('semweb/rdf_db.pl')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_actions')).
:- use_module(library('action_properties')).

:- rdf_meta
  task_actions(r, -),
  task_involving_object(r, r).

%% task_actions
%
% Read all sub-actions of a task, recursively including their sub-actions.
%
% @param Task Task identifier
% @param Actions List of action identifiers
% 
task_actions(Task, Actions) :-
  plan_subevents(Task, Actions).
  
%% task_involving_object
%
% Evaluates to true if task contains any action on object.
%
% @param Task Task containing any action on object
% @param Object Identifier of the object acted on
% 
task_involving_object(Task, Object) :-
  owl_subclass_of(Task, knowrob:'Action'),
  task_actions(Task, Actions),
  findall(Object2,
    (
      member(Action, Actions),
      action_property_has(Action, knowrob:objectActedOn, Object2)
    ),
    Objects2
  ),
  sort(Objects2, Objects),
  member(Object, Objects).
