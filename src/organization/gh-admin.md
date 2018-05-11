# Administration

To add new members to the organization and configure their permissions, we need an
administrator. Each year an administrator is designed who preferably has experience with
Git and GitHub in order to help his peers and resolve any Git problems they may face.

> This chapter is destined for the person who administrates the GitHub organization

The administrators for past years were:

- **2018** [@azerupi](https://github.com/azerupi) (Mathieu David)
- **2017** [@charlesvdv](https://github.com/charlesvdv) (Charles Vandevoorde)

You can contact them (most recent first preferably) if you are the chosen administrator for your team.
They will add you with the correct privileges.

## Creating a new repository

As administrator, your task is to setup GitHub and the repositories so that everyone can work on the 
project. The first thing you want to do is create a new repository for your year.

We do however recommend to base it off off the repository from previous year instead of starting from scratch.
As administrator, head over to the [organization's page](https://github.com/Ecam-Eurobot) to create a new repository.

![org](img/organization/org.png)

Fill in the name with `Eurobot-xxxx` where `xxxx` represents the year. You can add a description if you want. Leave all
the rest blank because we are going to push the code of previous year in the newly created repository.

![new repository](img/organization/new-repo.png)

You will land on the following page

![new repository created](img/organization/new-repo2.png)

Let's push the repository from last year into the newly created repository.
On the command line, clone last years repository. In this case, I will clone [Eurobot-2018](https://github.com/Ecam-Eurobot/Eurobot-2018)

```
git clone https://github.com/Ecam-Eurobot/Eurobot-2018.git
cd Eurobot-2018
```

Now you need to add the newly created repository as a remote. And verify that it was added correctly.

```
git remote add next-year https://github.com/Ecam-Eurobot/Eurobot-2019.git
git remote -v
```

Push the repository into the newly created repository

```
git push -u next-year master
```

If we refresh the GitHub page for the newly created repository, we can see something similar to below.
We can see (in the top left) that we are in the new repository and if we look at the `README` at the 
bottom we can see that it contains the files from previous year. We can now start to work in the new 
repository without affecting the old one.

![Repository pushed](img/organization/new-repo3.png)

> **Note:**  
> Now that the new repository is created, you can remove your local clone of the repository from last year.
> You will not need it anymore.

## Adding teams

Now that the repository is created, you need to add two **teams** to the Eurobot organization:

- **Eurobot \<year\>**: this team will contain all the members participating this year. We will give this team the privileges to push to any branch **except the master branch**.
- **Eurobot \<year\> Reviewers**: this team will be given more privileges. They will be able to review and accept [pull requests](https://help.github.com/articles/about-pull-requests/)
  to the master branch.

> **Note:**  
> The master branch should **always** be kept in a working state, this is the golden rule! As administrator, with help from the reviewers, it is your duty to make sure 
> that this rule is followed by everyone. The normal development process should be the following:
> 1. For any new development a new branch is created by the member that develops it
> 2. He implements the new feature / behavior
> 3. When done, he creates a pull request against the master branch
> 4. At least one reviewer reads the changes, makes sure that the code meets the quality guidelines and aproves the changes
> 5. Only then can the code be merged into the master branch.
>
> Resist the urge of merging code that hasn't been reviewed.

To create a new team, go to the GitHub organization: [Ecam-Eurobot](https://github.com/Ecam-Eurobot) and go to the "teams" tab.

![teams](img/organization/new-team-1.png)

Then click the **"New team"** button, fill in the name as mentioned above and then click **"Create the team"**.

### Give them correct permissions

Now that the teams are created, we need to give them the correct permissions. Go to the newly created repository, under **"Settings"**
go to the **"Collaborators & Teams"** section.

![teams](img/organization/new-team-2.png)

Then add the teams you created with the **write** permissions, like below.

![permissions](img/organization/permissions-1.png)

Now go to the **"Branches"** section and add the master branch as a protected branch.

![protected branch](img/organization/permissions-2.png)

And configure the protections like in the image below:

![protected branch](img/organization/permissions-3.png)

This will prevent anyone from commiting to the master branch directly **except** the administrator and the reviewers, who need push access to accept pull requests.
Don't abuse these privileges, it is always better to have your code reviewed by others, even if you are a badass programmer!

## Invite members
Now that everything is setup, we still need to invite the people who will be participating with you this year and assign them to the correct teams.
To do this, go to the organization's page again under the **"People"** tab and click on **"Invite member"**.

 ![members](img/organization/members-1.png)

Type in their GitHub user name and invite them. In the invitation, you can already assign them to the correct teams.

![members](img/organization/members-2.png)

Once the invitation has been sent, the invited user can accept the invitation by visiting the 
organization's page: [https://github.com/Ecam-Eurobot](https://github.com/Ecam-Eurobot).
