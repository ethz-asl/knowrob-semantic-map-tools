:- module(owl_string_parser, [
      owl_parse_string/1,              % +String
      owl_parse_string_from_file/1     % +File
   ]).

:- use_module(library('owl_parser.pl')).
:- use_module(library('semweb/rdf_db.pl')).
:- use_module(library('semweb/rdf_edit.pl')).
:- use_module(library('semweb/rdfs.pl')).
:- use_module(library('memfile.pl')).

%% owl_parse_string(+String)
%
% Parse an OWL string and load it into the local RDF database.
% 
% Resolves owl:imports and supports both the common URL formats
% (file paths, file:// or http://) and the package:// URLs used
% in ROS to reference files with respect to the surrounding ROS
% package.
%
% @param String OWL string
% 
owl_parse_string(String) :-
  (
    new_memory_file(Handle),
    open_memory_file(Handle, write, OutStream),
    write(OutStream, String),
      close(OutStream),
    open_memory_file(Handle, read, InStream),
    rdf_db:rdf_load(InStream, [format(xml)]),
    close(InStream),
    free_memory_file(Handle)
  ),
  (
    rdf(_, 'http://www.w3.org/2002/07/owl#imports', Import_URL),
    owl_parse_imports(Import_URL, [Import_URL|Imported]);
    true
  ).

owl_parse_imports(URL, Imported) :-
  (
    (
      sub_string(URL,0,4,_,'http'), !,
      http_open(URL,RDF_Stream,[]),
      rdf_load(RDF_Stream,[blank_nodes(noshare)]),
      close(RDF_Stream)
    ),
    assert(owl_file_loaded(URL));
    (
      sub_string(URL,0,7,_,'package'), !,
      sub_atom(URL, 10, _, 0, Path),
      atomic_list_concat(PathList, '/', Path),

      selectchk(Pkg, PathList, LocalPath),
      rospack_package_path(Pkg, PkgPath),

      atomic_list_concat([PkgPath|LocalPath], '/',  GlobalPath),

      rdf_load(GlobalPath,[blank_nodes(noshare)]),
      assert(owl_file_loaded(URL))
    );
    (
      rdf_load(URL,[blank_nodes(noshare)])
    ),
    assert(owl_file_loaded(URL))
  ),
  (
    rdf(_,'http://www.w3.org/2002/07/owl#imports',Import_URL),
    not( owl_file_loaded(Import_URL)),!,
    owl_parse_imports(Import_URL,[Import_URL|Imported]); 
    true
  ).

%% owl_parse_string_from_file(+File)
%
% Parse an OWL string read from a file and load it into the local RDF
% database.
%
% @param File Path to OWL file
%
owl_parse_string_from_file(File) :-
  open(File, read, Stream),
  read_stream_to_codes(Stream, Codes),
  string_codes(String, Codes),
  close(Stream),
  owl_parse_string(String),
  true.
