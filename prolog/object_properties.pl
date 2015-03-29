:- module(object_properties, [
      object_property/3         % ?Subject, +Property, ?Object
   ]).

:- use_module(library('semweb/rdf_db.pl')).

:- rdf_meta
  object_property(r, r, r).

%% object_property(?Subject, +Property, ?Object)
%
% Find subject or object relatated through object property
% 
% @param Subject Subject of the object property
% @param Property object property
% @param Object Object of the object property
% 
object_property(Subject, Property, Object) :-
  rdf_has(Subject, Property, Object).
