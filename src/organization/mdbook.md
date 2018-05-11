# mdBook

To generate this online documentation "book", we use a tool called [mdBook](https://github.com/rust-lang-nursery/mdBook).
This chapter will briefly introduce this tool in order for anyone to be able to contribute and improve this document.

The official documentation for mdBook can be found [here](https://rust-lang-nursery.github.io/mdBook/)

## Structure of a book

A book has the following structure:

```
book-test/
├── book.toml
├── book
└── src
    ├── chapter_1.md
    └── SUMMARY.md
```

The **`book.toml`** file contains the configuration options of the book. 
In this file you can find the title, the authors, but also an option to enable math equation rendering, etc.
For a list of the options, refer to the official documentation. The `book.toml` file also represents the 
root folder of the book. When we run mdBook, we need to either run it in that folder or point it to that folder.

In that same folder, you can find 2 folders: `src` and `book`. `src` contains the source files, written in [markdown](http://commonmark.org/help/).
When running the tool, it will take all the files in that directory and compile them into the book which is then stored in the `book` folder.

Finally, the most important file is the `SUMMARY.md` file. This file represents the table of content of the book, giving the hierarchy of all the
chapters and where to find their source files. The folliwing is an extract from this books summary file.

```md
# Summary

[Introduction](introduction.md)
- [Organization](organization/organization.md)
    - [GitHub](organization/github.md)
        - [Administration](organization/gh-admin.md)
    - [mdBook](organization/mdbook.md)
- [Mechanical](mechanical/mechanical.md)
    - [3D modeling with Fusion360](mechanical/fusion.md)
    - [3D printing](mechanical/3d-print.md)
    - [Mecanum wheels](mechanical/mecanum.md)
```

We can see that it is simply a set of nested markdown lists containing links to the source files.


## Markdown
The source files for the book are written in [Markdown](http://commonmark.org/), which is a very simple markup language.
You can [learn the basics in '60 seconds'](http://commonmark.org/help/).

### Syntax highlighting
To insert code blocks with syntax heighlighting, use triple backticks followed by the language name / abreviation:

    ```python
    import sys

    sys.exit(0)
    ```

This will generate the following:

```python
import sys

sys.exit(0)
```

### Images

In mdBook, paths to images should always be referenced from the `src` folder. So if you image is located in `src/img/my-image.png`,
you should use `![Some alt text](img/my-image.png)`.



## Generating the book

When you make changes locally, you probably want to see how it looks before making a commit.
You can install the tool for this. At the time of writing, [pre-built binaries] exist for Linux and MacOS,
but not for Windows.

For windows, I have compiled [a binary](https://github.com/Ecam-Eurobot/Tutorials/releases/tag/files) that you can use.
You can also **compile mdBook from source**, but this falls outside the scope of this tutorial.

When you have mdBook, add it to your path so that you can use it in the terminal / command line from anywhere.
For this, I will refer you to external documentation: [Linux / MacOS](https://unix.stackexchange.com/questions/26047/how-to-correctly-add-a-path-to-path) 
& [Windows](http://www.itprotoday.com/management-mobility/how-can-i-add-new-folder-my-system-path).

Now everything is setup, you should be able to open the terminal / command line and type `mdbook --help` and see the following:

```
mdBook --help
mdbook v0.1.5
Mathieu David <mathieudavid@mathieudavid.org>
Create a book in form of a static website from markdown files

USAGE:
    mdbook.exe [SUBCOMMAND]

FLAGS:
    -h, --help       Prints help information
    -V, --version    Prints version information

SUBCOMMANDS:
    build    Build the book from the markdown files
    clean    Delete built book
    help     Prints this message or the help of the given subcommand(s)
    init     Create boilerplate structure and files in the directory
    serve    Serve the book at http://localhost:3000. Rebuild and reload on change.
    test     Test that code samples compile
    watch    Watch the files for changes

For more information about a specific command, try `mdbook <command> --help`
Source code for mdbook available at: https://github.com/rust-lang-nursery/mdBook
```

The two command that are the most interesting for you are `build` and `serve`.

Running `mdbook build` in the folder where the `book.toml` is located will generate the book.
The generated book can then be found in the `book/` folder.

Running `mdbook serve` is even better, because it will watch the files for any changes and rebuild the book automatically.
On top of that, it serves the book on `http://localhost:3000` and automatically refreshes the browser after regenerating the book.
You launch it once, when you begin to write and forget about it. It is that simple.

## Hosting
When you visit the following address: [https://ecam-eurobot.github.io/Tutorials/](https://ecam-eurobot.github.io/Tutorials/) you can find this book online.
To host the website, we use GitHub. In the repository of the book, there is a branch called `gh-pages`. This is a special branch that can be used to host a
static website through GitHub.

When you push a new version of the generated book to this branch, it will be accessible online from the address above.

> **Note:**  
> **Don't push manually** to this branch. As explained below, a new version of the book is generated and pushed automatically on each new
> commit on the master branch! 

## Travis
We have setup [Travis](https://travis-ci.org/) to generate the book for each new commit on the master branch. This means that the hosted book is **always**
up to date, with a couple of minutes delay.

The travis configuration file looks like the following:

```yaml
sudo: false
dist: trusty
language: rust      # We want to download the Rust toolchain (because mdBook is written in Rust)
cache: cargo        # We want to cache the cargo folder to speed up the compilation of mdBook
rust:
  - stable
branches:
  only:
  - master          # We only want to execute Travis for commits on the master branch
before_script:
  - (cargo install mdbook --vers ^0.1.5 || true)    # Install mdBook (the || true trick is to avoid an error if it is already installed)
script:
- mdbook build      # Generate the book

# Deploy the book to GitHub Pages
deploy:
  provider: pages
  skip-cleanup: true
  github-token: $GH_TOKEN  # Set in travis-ci.org dashboard, marked secure
  keep-history: true
  local-dir: book
  on:
    branch: master
``` 