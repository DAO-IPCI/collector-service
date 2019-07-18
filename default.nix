{ pkgs ? import <nixpkgs> { }
, stdenv
, mkRosPackage
, robonomics_comm
}:

with pkgs.python37Packages;

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "collector_agent";
  version = "master";

  src = ./.;

  propagatedBuildInputs = [ robonomics_comm pkgs.python37Packages.psycopg2 pkgs.python37Packages.sqlalchemy ];

  meta = with stdenv.lib; {
    description = "The collector of data from energy source";
    homepage = http://github.com/vourhey/collector-agent;
    license = licenses.bsd3;
    maintainers = with maintainers; [ vourhey ];
  };
}
