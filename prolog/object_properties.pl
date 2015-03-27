:- module(object_properties, [
      hinged_to/2               % ?Subject, ?Object
   ]).

:- use_module(library('semweb/rdf_db.pl')).

%% hinged_to(?Subject, ?Object)
%
% Find subject or object relatated through object property knowrob:hingedTo
% 
% @param Subject Subject of the object property
% @param Object Object of the object property
% 
hinged_to(Subject, Object) :-
  rdf_has(Subject, knowrob:hingedTo, Object).
