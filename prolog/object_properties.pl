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
% @param Subject Subject of the object property (object instance or
%   class identifier)
% @param Property object property
% @param Object Object of the object property (object identifier)
% 
object_property(Subject, Property, Object) :-
  (
    rdf_has(Subject, Property, Object);
    rdfs_individual_of(Subject, Class),
    class_properties(Class, Property, Object);
    rdf_has(Subject, Property, literal(Value))
  ), !;
  class_properties(Subject, Property, Object), !.
