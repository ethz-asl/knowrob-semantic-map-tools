:- module(action_properties, [
      action_property/3                 % ?Action, ?Property, ?Object
   ]).

:- use_module(library('semweb/rdf_db.pl')).
:- use_module(library('knowrob_owl')).

:- rdf_meta
  action_property(r, r, r).

%% action_property(?Action, ?Property, ?Object)
%
% Reads the property for a TBOX action description
%
% @param Action Action class with a restriction on property
% @param Property Action property
% @param Object Value set as property for this action
% 
action_property(Action, Property, Object) :-
  owl_subclass_of(Action, knowrob:'Action'),
  owl_direct_subclass_of(Action, Sup),
  not(rdf_is_bnode(Action)),
  (
    (
      owl_direct_subclass_of(Sup, Sup2),
      owl_restriction(Sup2, restriction(Property, some_values_from(Object)))
    );
    owl_restriction(Sup, restriction(Property, some_values_from(Object)))
  ).
