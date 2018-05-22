/*
 * Copyright (C) 2018 Sertac Karaman
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *
 */

#ifndef __MU_CALCULUS_INCREMENTAL_MODEL_CHECKER_PT_
#define __MU_CALCULUS_INCREMENTAL_MODEL_CHECKER_PT_

// Parse Tree

#include <iostream>
#include <set>
#include <string>

#include <cstdio>
#include <cstdlib>

enum PT_type { PT_PRP = 1, PT_VAR, PT_LFP, PT_GFP, PT_SUC, PT_AND, PT_OR };

class PT_node;
class PT_prp;
class PT_var;
class PT_operator;

typedef std::set<PT_node *> subformulaeSet;
typedef std::set<PT_node *>::iterator subformulaeSet_it;

typedef std::set<int> propositionSet;
typedef std::set<int>::iterator propositionSet_it;

class PT_node {
public:
  PT_type type;
  PT_node *parent;
  virtual subformulaeSet getChildren();
  void printType();
};

class PT_prp : public PT_node {
public:
  int prp;
  virtual subformulaeSet getChildren();
};

class PT_var : public PT_node {
public:
  int var;
  virtual subformulaeSet getChildren();
};

class PT_operator : public PT_node {
public:
  subformulaeSet children;
  int boundVar;
  virtual subformulaeSet getChildren();
};

class ParseTree {
public:
  ParseTree();
  ~ParseTree();
  int parseFormula(std::string s);
  bool isEmpty();
  // 	subformulaeSet& getSubformulaeSuc ();
  // 	subformulaeSet& getSubformulaeMu ();
  // 	subformulaeSet& getSubformulaeNu ();
  // 	void SearchBranchForType (PT_node *node, subformulaeSet &sfSet, PT_type
  // nodeType);

  PT_node *getRoot();
  PT_node *getBoundFormula(PT_node *node_var);

  // Debug/Test/Visualization related functions
  void printParseTree(PT_node *ptnode);

  bool compareFormulaSize(
      PT_node *ptnode_a,
      PT_node *ptnode_b); // True if ptnode_a < ptnode_b in the parse tree

  // The following two functions are not used anymore...
  // 	bool booleanCheck (PT_node *root, subformulaeSet &satisfiedSFThis,
  //                        subformulaeSet &satisfiedSFNext, propositionSet
  //                        &satisfiedPrp);
  // 	bool fixedpointCheck (PT_node *root, subformulaeSet &satisfiedSFThis,
  //                           subformulaeSet &satisfiedSFNext, propositionSet
  //                           &satisfiedPrp, int var, bool lfp);

private:
  PT_node *root;
};

#endif
