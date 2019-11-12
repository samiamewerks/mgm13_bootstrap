# mgm13_bluetooth 

PURPOSE

Contains the code for the mgm13 bluetooth function. 

MAKING A LOCAL COPY

This repository is arranged as a snapshot of the Simplicity studio
IDE workspace. Thus you can do:

cd C:\Users\<user>\SimplicityStudio\v4_workspace

(where "\<user\>" is your user name)

git clone https://github.com/samiamewerks/mgm13_bluetooth.git

And it will create the directory:

mgm13_bluetooth

Containing the repository. Alternately you can name the repository as:

git clone https://github.com/samiamewerks/mgm13_bluetooth.git myrepo

And the resulting repository is named "myrepo".

ACTIVATING THE PROJECT IN SIMPLICITY STUDIO

In the Simplicity IDE, select file->import

In the "project search" dialog, you will either see the new project in
"detected projects", or if not, hit "browse" and find the project directory
created above. Then your project path should be in the "select project to
import" window. If the project is in the list of "detected projects" just
double click that (Studio remembers previous projects, even if you deleted
them).

Now hit next, you should see "build configuration", and all of the fields
should be filled out. Hit "next" again, you will see the "project 
configuration" dialog with "project name" filled out with the name of your
project, but with a "_2" added to the end. You need to remove this ending,
and use the existing directory (otherwise you get an error).
