# Summative Research Project / Technical Report

For this assignment, you will be researching an algorithm or data structure not covered in the course. 
You will be writing a report on the algorithm or data structure, and 
you will be implementing it in **a programming language of your choice**. You will be presenting your 
research and implementation as part of the repository submission. 

## Deliverables:
* The report should be written in markdown, and as your [README.md](../README.md) file for the repository. 
* All code written should be provided
* Tests for code should be provided
* Any related files and evidence of running the algorithm/datastructure should be provided. For example: you capture the output of your run into text files. 

From your deliverables, we should not only be able to see a valid implementation, but that you confirmed and fully
tested it in addition to the report.

## Details:

This is an open ended research project with the condition it must be an algorithm or data structure not covered in the course.
With that said, it can be difficult to select a topic. Here are some suggestions:

* B+ Trees
* AVL Tree
* Cartesian Tree
* R+ Tree
* Red–black tree
* Sorted Hashmap
* Hashmap with Cuckoo Hashing
* (Really, you can look at the java collections interface and get a bunch of ideas of what to implement)

For some algorithms, you may want to consider implementing a specific algorithm, or a specific type of algorithm.

* A* Search Algorithm
* Alpha-beta pruning (or just minimax)
* Ant Colony Optimization
* Basic RSA implementation (small key sizes)
* Best First Search
* Collision Detection
* Fast Fourier Transform (FFT)
* Ford–Fulkerson algorithm (flow networks)
* Genetic Algorithm for optimization problems
* Huffman coding algorithm
* Jump Point Search
* KnapSack
* Kruskal's or Prim's algorithm
* Lagged Fibonacci generator (or other Pseudorandom number generator)
* Line Segment Intersection
* Minimum Bounding Box
* Minimum Spanning Tree
* Tim Sort
* Trigram Search



Here are some overly broad topic lists.

* [https://en.wikipedia.org/wiki/List_of_data_structures](https://en.wikipedia.org/wiki/List_of_data_structures)
* [https://en.wikipedia.org/wiki/List_of_algorithms](https://en.wikipedia.org/wiki/List_of_algorithms)

> [!TIP]
> You should look up algorithms related to your field of interest! Or just explore, either is fine. The important part is try to keep it simple. 

### Selection Guidelines

It is easy to select something either too large or too small. The balance is if you end up doing a lot of coding, we will 
look more at the code than the report. But if your coding is relatively simple (as single function with a few test
cases), then we will expect a more detailed report/research surrounding it. 

Part of programming is often researching the best way to do something. For this assignment, you will be demonstrating your ability
to research and implement an algorithm or data structure.  

> [!CAUTION]
> You must implement the algorithm yourself. For example, you cannot use Java's Sorted Hashmap and say you implemented it by testing it. You could look at Java's SortedHashmap since the code is freely available, and then implement your own version of it in another language that is fine. 


> Feel free to reach out to ask if something is of the appropriate size! Also feel free to talk about algorithms or data structures in the general channel on MS Teams!! Asking questions about your data structure / algorithm will help others. 

## Empirical Analysis

You will be providing a detailed analysis of the algorithm or data structure. However, it can also be very difficult to figure out the best way to analyze an algorithm or data structure. We have already shown using across multiple homeworks and the Midterm comparison analysis by looking at the speed between algorithms or the speed between various implementations of the same algorithm. 

During your team activities, you have also look at collisions for hash functions, and used that as a means to compare hash functions. There are other ways to look at an analysis also, including documenting the running of the algorithm itself. For example, adding a large number of items to an AVL, and then documenting how many swaps are needed to keep it balanced, and then comparing those swaps to the number of searches operations.  Another way is to show the efficiency of the algorithm given a certain dataset size, and then discuss its scalability. For some algorithms, you will find are good at smaller sizes, but they fall into a category called NP or NP-Complete at higher levels. That is fine! You do not need to be trying to solve it for the large cases (those are actually very difficult to solve - and the heart of the discussion in 5800: P vs NP).

We are sure that you can come up with other means such as graphs, visualizations, and more, but the important part is that you are telling us the 'story' behind it in a data driven way. It needs to help you understand the algorithm, and help the reader better understand the algorithm.  Yes, an online webpage can be a visualization (if you make one). If you do that, make sure you include it in your report / or at least screen shots of it running. We would consider that part of the analysis even if it isn't an analysis in the traditional sense.

> We suggest you think about this *BEFORE* you implement the algorithm and feel free to ask for ideas. 

### Theoretical Analysis
Another major component of this paper is you providing a theoretical analysis of the algorithm. This will involve using mathtex (latex math) inside of the markdown, proving to us the runtime and even proving correctness at the highest level. However, it needs to be obvious you understand the analysis. If it looks like you copied and pasted something you found, that is considered plagiarism. As such, include a discussion that describes what we are seeing or inline comments (depending on how you do it). Don't forget the pseudo-code!   

> [!IMPORTANT]
> Overall, an approach to take with this paper - assume the reader is your employer who asked you to review the algorithm. They have a technical background, but need someone to go through the details, so they can make a business decision to include it in the application or not. 

## 📝 Grading Rubric

You will submit your project as a link to your repository on Canvas. Your repository should contain all of the deliverables listed above, and it needs to be in the github classroom repo that is generated from the github classroom link. This will give TAs full access to the project without needing to make it public. 

The rubric will be as follows.

| Category | Exceeds (4pts) | Meets (3pts) | Approaching (2pts) | Learning (1pt) | Missing (0pts) | 
| --- | --- | --- | --- | --- | --- |
| **Code Quality** | Code is written correctly, based on the language norms including memory management and small concise functions. Tests included to show validity. Sample runs included. | Code is both documented and test files included. | Code exists, is documented OR tested by including test files for all the code. | Code exists but is not documented or tested. Or it is evident code does not run correctly. | No code or evident it cannot compile. |    
| **Writing / Grammar** | Report is easy to read, follows proper formatting guidelines, matches correct audience. | Report uses grammar at a college level. | Difficult to read due to poor sentence structures and wording choices. | Difficult to read. Report uses basic grammar, may have misspelling and obvious grammar mistakes. | No report. |
| **Visuals** (chart, graph, math notation, etc) |  Visuals are informative, and described properly in the paper writing, adding to the overall report. | Visuals are informative, and help clarify the report. | Has a visual but no reference to visual in the paper, and out of place. | Student includes visuals, but they are distracting and not informative. | No visuals. |
| **Algorithm Background** | Student provides a detailed background of the algorithm or data structure, including its history, and how it is used today. It also provides a high level overview of how the algorithm works. | Student provides a detailed background of the algorithm or data structure, how it works, but may be missing history or how it is used. Pseudo code is provided for the algorithm, or for data structures, pseudo code is provided for key features. | Student provides a background of the algorithm or data structure, but missing details and clarity. | Student talks about the algorithm at a high level overview. | No discussion of the algorithm. |
| **Theoretical Analysis** | In addition to detailed Runtime and space analysis, student is able to provide a proof of correctness for the algorithm (using loop invariant, induction, proof by contradiction, etc). Similar option, is they provide detailed math on how the runtime analysis is derived. | Student provides a detailed analysis of the algorithm or data structure, including its efficiency such as BigO or other appropriate mathematical notation. Details situations for best/worst/average case if correct for the algorithm type, and if appropriate provides comparisons to similar algorithms. | Provides some analysis of the algorithm or data structure, but may be missing details or clarity such as changes in speed based on applications/cases. | Student talks about the algorithm at a high level overview, may provide discussion of other algorithms. | No analysis of the algorithm. |
| **Empirical Analysis** | Contains detailed empirical data that matches the theoretical run times. Takes into account languages limitations, and addresses how those limitations influence/bias the analysis. | Contains empirical data that is supported by visuals, and discussion. May have errors in assumptions / validity errors without addressing the concerns. | Contains a discussion of the generated data, but discussion may not be clearly defined. | Contains empirical data, no discussion or addressing assumptions. | No empirical data. |
| **Implementation** | Provides key details about the implementation, including code snippets and any other information such as references to other languages if they were used as samples. Could be easily reproduced based on writeup.|  Provides details and code snippets on the implementation, but may be sparse / difficult to follow. | Details on the implementation and no code snippets, or code snippets and no discussion. | Provides confusing details on the implementation / invalid details / misinformation. | No implementation details. |
| **References** | Contains both inline citations and end of paper references. Has at least 2 peer reviewed academic works related to algorithm or data structure, and 5 references at a minimum. | Has at least 3 references. Contains end of paper references and inline citations. | Has less than three references, inline and end of paper references  | Contains less than three references, and has only inline citations or end of paper references (not both).  | No references. |

Total Points: 32

Please note: As we better refine the rubric, it may be updated on wording. Double check the wording in canvas before submission as that is the most accurate version. 


## 📚 Resources
* [ACM Journal Access](https://login.ezproxy.neu.edu/) Login via this link to ACM (using your NEU credentials) to get free access to the ACM (association for computing machinery) journals. *This is arguably your best spot for peer reviewed journals in computer science.*
* [NEU Library Journal Finder](https://onesearch.library.northeastern.edu/discovery/jsearch?vid=01NEU_INST:NU) Some journals you may need to be on the Northeastern VPN to access.
* [Google Scholar](https://scholar.google.com/) Search focused on academic works.
* [NEU Writing Center](https://cssh.northeastern.edu/writingcenter/tutoring/first-visit/) - highly recommend you get your report evaluated! As  graduate student, worth going to a writing center at least once to get feedback, and better direct your writing style. 
* [NEU Research Tutorials](https://subjectguides.lib.neu.edu/researchtutorials)
* [Research Report](https://researchmethod.net/research-report/) - Note, this project is more of a technical report / white paper.  
* [Writing Research Reports: Tips and Strategies](https://universitymanual.com/blog/writing-research-reports-tips-strategies/)
