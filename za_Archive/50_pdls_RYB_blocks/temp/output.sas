begin_version
3
end_version
begin_metric
1
end_metric
16
begin_variable
var0
-1
2
Atom atpose(v2, v6)
NegatedAtom atpose(v2, v6)
end_variable
begin_variable
var1
0
2
Atom unsafeapproach(v4, v7, v10)
NegatedAtom unsafeapproach(v4, v7, v10)
end_variable
begin_variable
var2
0
2
Atom unsafeapproach(v5, v8, v11)
NegatedAtom unsafeapproach(v5, v8, v11)
end_variable
begin_variable
var3
-1
2
Atom atpose(v4, v7)
NegatedAtom atpose(v4, v7)
end_variable
begin_variable
var4
0
2
Atom unsafeapproach(v2, v6, v9)
NegatedAtom unsafeapproach(v2, v6, v9)
end_variable
begin_variable
var5
0
2
Atom unsafetraj(v13)
NegatedAtom unsafetraj(v13)
end_variable
begin_variable
var6
0
2
Atom unsafetraj(v15)
NegatedAtom unsafetraj(v15)
end_variable
begin_variable
var7
0
2
Atom unsafetraj(v17)
NegatedAtom unsafetraj(v17)
end_variable
begin_variable
var8
-1
2
Atom atpose(v5, v8)
NegatedAtom atpose(v5, v8)
end_variable
begin_variable
var9
-1
2
Atom atgrasp(v2, v9)
NegatedAtom atgrasp(v2, v9)
end_variable
begin_variable
var10
-1
2
Atom atgrasp(v4, v10)
NegatedAtom atgrasp(v4, v10)
end_variable
begin_variable
var11
-1
2
Atom atgrasp(v5, v11)
NegatedAtom atgrasp(v5, v11)
end_variable
begin_variable
var12
-1
2
Atom handempty()
NegatedAtom handempty()
end_variable
begin_variable
var13
-1
4
Atom atconf(v1)
Atom atconf(v12)
Atom atconf(v14)
Atom atconf(v16)
end_variable
begin_variable
var14
-1
2
Atom canmove()
NegatedAtom canmove()
end_variable
begin_variable
var15
0
2
Atom holding(v2)
NegatedAtom holding(v2)
end_variable
0
begin_state
1
1
1
1
1
1
1
1
1
1
1
1
0
0
0
1
end_state
begin_goal
1
15 0
end_goal
18
begin_operator
move_free v1 v12 v19
1
12 0
2
0 13 0 1
0 14 0 1
1000
end_operator
begin_operator
move_free v1 v14 v22
1
12 0
2
0 13 0 2
0 14 0 1
1000
end_operator
begin_operator
move_free v1 v16 v27
1
12 0
2
0 13 0 3
0 14 0 1
1000
end_operator
begin_operator
move_free v12 v1 v18
1
12 0
2
0 13 1 0
0 14 0 1
1000
end_operator
begin_operator
move_free v12 v14 v23
1
12 0
2
0 13 1 2
0 14 0 1
1000
end_operator
begin_operator
move_free v12 v16 v29
1
12 0
2
0 13 1 3
0 14 0 1
1000
end_operator
begin_operator
move_free v14 v1 v20
1
12 0
2
0 13 2 0
0 14 0 1
1000
end_operator
begin_operator
move_free v14 v12 v21
1
12 0
2
0 13 2 1
0 14 0 1
1000
end_operator
begin_operator
move_free v14 v16 v28
1
12 0
2
0 13 2 3
0 14 0 1
1000
end_operator
begin_operator
move_free v16 v1 v24
1
12 0
2
0 13 3 0
0 14 0 1
1000
end_operator
begin_operator
move_free v16 v12 v26
1
12 0
2
0 13 3 1
0 14 0 1
1000
end_operator
begin_operator
move_free v16 v14 v25
1
12 0
2
0 13 3 2
0 14 0 1
1000
end_operator
begin_operator
pick v2 v6 v9 v12 v13
3
13 1
4 1
5 1
3
0 9 -1 0
0 14 -1 0
0 12 0 1
1000
end_operator
begin_operator
pick v4 v7 v10 v14 v15
3
13 2
1 1
6 1
3
0 10 -1 0
0 14 -1 0
0 12 0 1
1000
end_operator
begin_operator
pick v5 v8 v11 v16 v17
3
13 3
2 1
7 1
3
0 11 -1 0
0 14 -1 0
0 12 0 1
1000
end_operator
begin_operator
place v2 v6 v9 v12 v13
3
13 1
4 1
5 1
4
0 9 0 1
0 0 -1 0
0 14 -1 0
0 12 -1 0
1000
end_operator
begin_operator
place v4 v7 v10 v14 v15
3
13 2
1 1
6 1
4
0 10 0 1
0 3 -1 0
0 14 -1 0
0 12 -1 0
1000
end_operator
begin_operator
place v5 v8 v11 v16 v17
3
13 3
2 1
7 1
4
0 11 0 1
0 8 -1 0
0 14 -1 0
0 12 -1 0
1000
end_operator
22
begin_rule
1
9 0
15 1 0
end_rule
begin_rule
1
0 0
1 1 0
end_rule
begin_rule
1
0 0
2 1 0
end_rule
begin_rule
1
0 0
5 1 0
end_rule
begin_rule
1
0 0
6 1 0
end_rule
begin_rule
1
0 0
7 1 0
end_rule
begin_rule
2
0 1
3 1
2 0 1
end_rule
begin_rule
3
0 1
3 1
8 1
5 0 1
end_rule
begin_rule
3
0 1
3 1
8 1
6 0 1
end_rule
begin_rule
3
0 1
3 1
8 1
7 0 1
end_rule
begin_rule
2
0 1
8 1
1 0 1
end_rule
begin_rule
1
3 0
4 1 0
end_rule
begin_rule
1
3 0
2 1 0
end_rule
begin_rule
1
3 0
5 1 0
end_rule
begin_rule
1
3 0
6 1 0
end_rule
begin_rule
1
3 0
7 1 0
end_rule
begin_rule
2
3 1
8 1
4 0 1
end_rule
begin_rule
1
8 0
4 1 0
end_rule
begin_rule
1
8 0
1 1 0
end_rule
begin_rule
1
8 0
5 1 0
end_rule
begin_rule
1
8 0
6 1 0
end_rule
begin_rule
1
8 0
7 1 0
end_rule
