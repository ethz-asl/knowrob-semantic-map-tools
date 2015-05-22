:- module(data_properties, [
      data_property/3,             % ?Subject, +Property, -Value
      data_property/4              % ?Subject, +Property, -Value, +DefaultValue
   ]).

:- use_module(library('semweb/rdf_db.pl')).

:- rdf_meta
  data_property(r, r, -),
  data_property(r, r, -, +).

%% data_property(?Subject, +Property, -Value)
%
% Find data property value for subject
% 
% @param Subject Subject of the data property (object instance or
%   class identifier)
% @param Property Data property
% @param Value Value of the data property
% 
data_property(Subject, Property, Value) :-
  (
    rdf_has(Subject, Property, literal(type(_, Value)));
    rdfs_individual_of(Subject, Class),
    class_properties(Class, Property, literal(type(_, Value)));
    rdf_has(Subject, Property, literal(Value)); 
    rdfs_individual_of(Subject, Class),
    class_properties(Class, Property, literal(Value))
  ), !;
  class_properties(Subject, Property, literal(type(_, Value))), !.

%% data_property(?Subject, +Property, -Value, +DefaultValue)
%
% Find data property value for subject, with default value
% 
% @param Subject Subject of the data property (object instance or
%   class identifier)
% @param Property Data property
% @param Value Value of the data property
% @param DefaultValue Default value of the data property
% 
data_property(Subject, Property, Value, DefaultValue) :-
  data_property(Subject, Property, Value) ->
    true;
    Value = DefaultValue.
