#include <rtthread.h>
#include <ctype.h>
#ifdef RT_USING_INI_HELPER
#ifdef RT_USING_DFS
#include <dfs.h>
#include <dfs_file.h>
#include <dfs_posix.h>
#include <stdio.h>
#endif

#include "util_ini.h"

/* Maximum number of characters expected on a line.
It is only used by the ini_putf() function.
*/
#define MAX_LINE        64

/* Various error codes */
#define SUCCESS                       1
#define FILE_CREATED                  0
#define NO_SUCH_FILE                  -1
#define OUT_OF_MEMORY                 -2
#define MISSING_END_BRACE             -3
#define EMPTY_SECTION                 -4
#define EXPECTED_EQUALS             	-5
#define EXPECTED_END_OF_STRING        -6
#define ER_FOPEN                        -7
#define BAD_SYMBOL                      -8
#define EXPECTED_PARAMETER            -9
#define EXPECTED_VALUE                -10

#define jmp_buf rt_int32_t
#define setjmp(x)	 (0)
#define longjmp(err,ecode) do{rt_kprintf(ini_errstr(ecode)); if(((ecode) != EXPECTED_EQUALS)) RT_ASSERT(0);}while(0)
//#define isspace(c)           ((c) == ' ' || (c) == '\f' || (c) == '\n' || (c) == '\r' || (c) == '\t' || (c) == '\v')
//#define isgraph(c)						((c) != ' ')
#define T_END        0
#define T_VALUE        1

static int get_token(const char **tp, const char **tstart, const char **tend, int *line, jmp_buf err);
static char *get_string(const char *tstart, const char *tend, jmp_buf err);

rt_int32_t rt_stricmp(const char *cs, const char *ct)
{
    while (*cs && tolower(*cs) == tolower(*ct))
        cs++, ct++;

    return (*cs - *ct);
}
RTM_EXPORT(rt_stricmp);
char *rt_strchr(const char* cs,char c)
{
	char* ct = RT_NULL;
	while(*cs)
	{
		if (c == *cs)
		{
			ct = (char*)cs;
			break;
		}
		cs++;
	}
	return ct;
}
RTM_EXPORT(rt_strchr);


/* Reads an entire file into a dynamically allocated memory buffer.
 * The returned buffer needs to be free()d afterwards
 */

static char *ini_readfile(const char *fname) 
{
	struct stat st;
	int fd = -1;
	long len,r;
	char *str;
	if( 0 >(fd = open(fname, O_RDONLY,0)))  
		return RT_NULL;
	if (fstat(fd, &st)<0)
	{
		close(fd);
		return RT_NULL;
	}
	len = st.st_size;
	if(!(str = (char*)rt_malloc(len+2)))
	{
		close(fd);
		return RT_NULL;  
	}
	r = read(fd,str,len);
	if (r != len)
	{
		rt_free(str);
		close(fd);
		return RT_NULL;  
	}
	str[len] = '\0';
	close(fd);
	return str;
//    FILE *f;
//    long len,r;
//    char *str;
//    
//    if(!(f = fopen(fname, "rb")))    
//        return RT_NULL;
//    
//    fseek(f, 0, SEEK_END);
//    len = ftell(f);
//    rewind(f);
//    
//    if(!(str = (char*)rt_malloc(len+2)))
//        return RT_NULL;    
//    r = fread(str, 1, len, f);
//    
//    if(r != len) {
//        rt_free(str);
//        return RT_NULL;
//    }
//    
//    fclose(f);    
//    str[len] = '\0';
//    return str;
}


/*****************************/
const char *ini_errstr(int err)
{
    switch(err)
    {
        case SUCCESS : return "Success";
        case FILE_CREATED: return "New INI object created";
        case NO_SUCH_FILE: return "Unable to open file";
        case OUT_OF_MEMORY: return "Out of memory";
        case MISSING_END_BRACE: return "Missing ']' at end of section";
        case EMPTY_SECTION: return "Empty [] for section";
        case EXPECTED_EQUALS : return "Expected an '='/':'";
        case EXPECTED_END_OF_STRING : return "Expected an end of string";
        case ER_FOPEN : return "Unable to open file";
        case BAD_SYMBOL : return "Bad symbol";
        case EXPECTED_PARAMETER : return "Expected a parameter (or section)";
        case EXPECTED_VALUE : return "Expected a value";
    }
    return "Unknown";
}

/** Configurable parameters *************************************************/

/*
 *    Recursively adds sections to the tree of sections
 */
static void insert_section(ini_section *r, ini_section *n) {
    RT_ASSERT(r);
    RT_ASSERT(n);

    if(rt_stricmp(r->name, n->name) < 0) {
        if(!r->left)
            r->left = n;
        else
            insert_section(r->left, n);
    } else {
        if(!r->right)
            r->right = n;
        else
            insert_section(r->right, n);
    }
}
/*
 *    Searches a tree of pairs for a specific parameter
 */
static ini_pair *find_pair(ini_pair *root, const char *name) 
{
    int c;

    if(!root) return RT_NULL;

    c = rt_stricmp(root->param, name);
    if(c == 0)
        return root;
    else if(c < 0)
        return find_pair(root->left, name);
    else
        return find_pair(root->right, name);
}
/*
 *    Searches for a specific section
 */
static ini_section *find_section(ini_section *root, const char *name) {
    int c;

    if(!root) return RT_NULL;

    c = rt_stricmp(root->name, name);
    if(c == 0)
        return root;
    else if(c < 0)
        return find_section(root->left, name);
    else
        return find_section(root->right, name);
}
/*
 *    Creates a new section, and adds it to a tree of sections
 */
static ini_section *add_section(ini_section **root, char *name) {
    ini_section *n;

    RT_ASSERT(root);
    RT_ASSERT(name);

    n = find_section(*root, name);
    if(n) {
        rt_free(name);
        return n;
    }

    n = (ini_section*)rt_malloc(sizeof *n);
    if(!n) return RT_NULL;

    n->name = name;
		n->valid = 1;
    n->fields = RT_NULL;
    n->left = n->right = RT_NULL;
    if(*root)
        insert_section(*root, n);
    else
        *root = n;

    return n;
}
/*
 *    Inserts a new pair n into a pair tree p
 */
static void delete_pair(ini_pair *p)
{
	p->valid = 0;
}
static void delete_section(ini_section *s)
{
	s->valid = 0;
}
static void insert_pair(ini_pair *p, ini_pair *n) {
    if(rt_stricmp(p->param, n->param) < 0) {
        if(!p->left)
            p->left = n;
        else
            insert_pair(p->left, n);
    } else {
        if(!p->right)
            p->right = n;
        else
            insert_pair(p->right, n);
    }
}
/*
 *    Adds a parameter-value pair to section s
 */
static ini_pair *add_pair(ini_section *s, char *p, char *v) {
    ini_pair *n;

    RT_ASSERT(s);

    n = (ini_pair*)rt_malloc(sizeof *n);
    if(!n) return RT_NULL;

    n->param = p;
    n->value = v;
		n->valid = 1;
    n->left = n->right = RT_NULL;

    if(!s->fields)
        s->fields = n;
    else
        insert_pair(s->fields, n);
		s->valid = 1;
    return n;
}
/** Functions for memory deallocation ***************************************/

/*
 *    Free's a tree of parameter-value pairs
 */
static void free_pair(ini_pair *p) {
    if(!p) return;

    free_pair(p->left);
    free_pair(p->right);

    rt_free(p->param);
    rt_free(p->value);
    rt_free(p);
}
/*
 *    Free's all the memory allocated to a ini_section s
 */
static void free_section(ini_section *s) {
    if(!s) return;

    free_pair(s->fields);
    free_section(s->left);
    free_section(s->right);

    rt_free(s->name);
    rt_free(s);
}
/*
 *    Free's all the memory allocated to a ini_file object in ini_read()
 */
void ini_free(struct ini_file *ini) {
    if(!ini) return;
    free_pair(ini->globals);
    free_section(ini->sections);
    rt_free(ini);
}
/** Parsing functions *******************************************************/

static struct ini_file *make_ini() 
{
    struct ini_file *ini = (struct ini_file*)rt_malloc(sizeof *ini);
    if(!ini) return RT_NULL;
    ini->globals = RT_NULL;
    ini->sections = RT_NULL;
    return ini;
}
/*
 *    Reads an INI file and returns it as a ini_file object.
 *    If filename is RT_NULL, an empty ini_file object is created and returned.
 */
struct ini_file *ini_read(const char *filename, int *err, int *line) {
    if(line) *line = 0;
    if(!filename)
    {
        if(err) *err = FILE_CREATED;
        return make_ini();
    } 
    else 
    {
			struct ini_file * ini;
        char *text = ini_readfile(filename);
        if(!text) {
            if(err) *err = NO_SUCH_FILE;
            return RT_NULL;
        }
        ini = ini_parse(text, err, line);
        rt_free(text);
        return ini;
    }
}
extern char *rt_strchr(const char* cs,char c);
static int get_token(const char **tp, const char **tstart, const char **tend, int *line, jmp_buf err) {
    /* *tstart points to the start of the token, while *tend points one char past the end */

    const char *t = *tp;
    int tok = T_END;

    RT_ASSERT(tp && tstart && tend);

whitespace:
    while(isspace(t[0])) {
        if(t[0] == '\n' && line)
            (*line)++;
        t++;
    }
    if(t[0] == ';' || t[0] == '#') {
        while(t[0] != '\n' && t[0] != '\0')
            t++;
        goto whitespace;
    }

    *tstart = t;
    *tend = t;
    if(t[0]) {
        if(rt_strchr("[]:=", t[0])) {
            tok = *t++;
        } else if(isgraph(t[0]) && !rt_strchr("\"'[];#=", t[0])) {//wickkid++=
            while(isgraph(t[0]) && !rt_strchr("\"'[];#=", t[0])) {//wickkid++=
                t++;
            }
            *tend = t;
            tok = T_VALUE;
        } else if(t[0] == '\"' || t[0] == '\'') {
            char delim = t[0];
            if(t[1] == delim && t[2] == delim) {
                /* """Python style long strings""" */
                t += 3;
                *tstart = t;
                while(!(t[0] == delim && t[1] == delim && t[2] == delim)) {
                    if(t[0] == '\0') {
                        longjmp(err, EXPECTED_END_OF_STRING);
                    } else if(t[0] == '\\')
                        t++;
                    t++;
                }
                *tend = t;
                t+=3;
            } else {
                *tstart = ++t;
                while(t[0] != delim) {
                    if(t[0] == '\0' || t[0] == '\n') {
                        longjmp(err, EXPECTED_END_OF_STRING);
                    } else if(t[0] == '\\')
                        t++;
                    t++;
                }
                *tend = t++;
            }
            tok = T_VALUE;
        } else {
            /* Unrecognized token */
            longjmp(err, BAD_SYMBOL);
        }
    }

    *tp = t;
    return tok;
}

static char *get_string(const char *tstart, const char *tend, jmp_buf err) 
{
    char *string, *s;
    const char *i;

    RT_ASSERT(tend > tstart);
    string = (char*)rt_malloc(tend - tstart + 1);
    if(!string)
        longjmp(err, OUT_OF_MEMORY);

    for(i = tstart, s = string; i < tend; i++) {
        if(i[0] == '\\') {
            switch(*++i) {
                case '\\':
                case '\'':
                case '\"': *s++ = i[0]; break;
                case 'r': *s++ = '\r'; break;
                case 'n': *s++ = '\n'; break;
                case 't': *s++ = '\t'; break;
                case '0': *s++ = '\0'; break;
                default: break;
            }
        } 
//				else if (i[0] == '=')
//				{//wickkid ++
//					break;
//				}
				else {
            *s++ = i[0];
        }
    }
    RT_ASSERT(s - string <= tend - tstart);
    s[0] = '\0';
    return string;
}

struct ini_file *ini_parse(const char *text, int *err, int *line) {
    jmp_buf on_error;
    int e_code;

    struct ini_file *ini = RT_NULL;
    ini_section *cur_sec = RT_NULL;
    const char *tstart, *tend;

    int t;

    if(err) *err = SUCCESS;
    if(line) *line = 1;

    ini = make_ini();

    if((e_code = setjmp(on_error)) != 0) {
        if(err) *err = e_code;
        ini_free(ini);
        return RT_NULL;
    }

    while((t = get_token(&text, &tstart, &tend, line, on_error)) != T_END) {
        if(t == '[') {
            char *section_name;
            if(get_token(&text, &tstart, &tend, line, on_error) != T_VALUE) {
                longjmp(on_error, EMPTY_SECTION);
            }

            section_name = get_string(tstart, tend, on_error);

            cur_sec = add_section(&ini->sections, section_name);
            if(!cur_sec)
                longjmp(on_error, OUT_OF_MEMORY);

            if(get_token(&text, &tstart, &tend, line, on_error) != ']') {
                longjmp(on_error, MISSING_END_BRACE);
            }

        } else if (t == T_VALUE ) {
            char *par, *val;
            par = get_string(tstart, tend, on_error);
            t = get_token(&text, &tstart, &tend, line, on_error);
            if(t != '=' && t != ':') {
                longjmp(on_error, EXPECTED_EQUALS);
            }
            if(get_token(&text, &tstart, &tend, line, on_error) != T_VALUE) {
                longjmp(on_error, EXPECTED_VALUE);
            }
            val = get_string(tstart, tend, on_error);

            if(cur_sec)
                add_pair(cur_sec, par, val);
            else {
                /* Add the parameter and value to the INI file's globals */
                ini_pair *pair;
                if(!(pair = (ini_pair*)rt_malloc(sizeof *pair)))
                    longjmp(on_error, OUT_OF_MEMORY);

                pair->param = par;
                pair->value = val;
								pair->valid = 1;

                pair->left = pair->right = RT_NULL;

                if(!ini->globals)
                    ini->globals = pair;
                else
                    insert_pair(ini->globals, pair);
            }


        } else
            longjmp(on_error, EXPECTED_PARAMETER);
    }

    return ini;
}

/** Printing functions ******************************************************/

static void string_to_file(int fd/* FILE *f*/, const char *s) {
	int len = 0;
	if (!s) return;
	if ( -1 == fd) return;
	
	len = rt_strlen (s);
	if (len > 0)
	{
		write(fd, s, len);
	} 
//    fputc('\"', f);
//    for(; s[0]; s++) {
//        switch(s[0]) {
//            case '\n': fputs("\\n",f); break;
//            case '\r': fputs("\\r",f); break;
//            case '\t': fputs("\\t",f); break;
//            case '\"': fputs("\\\"",f); break;
//            case '\'': fputs("\\\'",f); break;
//            case '\\': fputs("\\\\",f); break;
//            default : fputc(s[0], f); break;
//        }
//    }
//    fputc('\"', f);
}

/*
 *    Recursively prints a tree of ini_pairs
 */
static void write_pair(ini_pair *p, int fd/* FILE *f*/) {
	if(!p) return;
	if (p->valid)
	{
		string_to_file(fd, p->param);
		string_to_file(fd, "=");
		string_to_file(fd, p->value);
		string_to_file(fd, "\r\n");
	}
	write_pair(p->left, fd);
	write_pair(p->right, fd);
	
/*
    string_to_file(f, p->param);
    fputs(" = ", f);
    string_to_file(f, p->value);
    fputc('\n', f);

    write_pair(p->left, f);
    write_pair(p->right, f);*/
}

/*
 *    Recursively prints a tree of INI sections
 */
static char check_section(ini_pair *p)
{
	if (!p)
		return 0;
	if (p->valid)
		return 1;
	if (check_section(p->left))
		return 1;
	return check_section(p->right);
}
static void write_section(ini_section *s, int fd/* FILE *f*/) {
  if(!s) return;
	if (check_section(s->fields) && s->valid)
	{
		string_to_file(fd, "[");
		string_to_file(fd, s->name);
		string_to_file(fd, "]\r\n");
		write_pair(s->fields, fd);
	}
	write_section(s->left, fd);
	write_section(s->right, fd);
	
/*
    fputs("\n[", f);
    string_to_file(f, s->name);
    fputs("]\n", f);

    write_pair(s->fields, f);*

    /* The akward sequence is to ensure that values are not written sorted */
/*
    write_section(s->left, f);
    write_section(s->right, f);*/
}

/*
 *    Saves all the sections and parameters in an ini_file to a file.
 *    If fname is RT_NULL, it is written to stdout.
 */
int ini_write(struct ini_file *ini, const char *fname) {
	int fd = -1;
    
    if(fname) {
		fd = open(fname, O_WRONLY | O_CREAT | O_TRUNC ,0);
    if(fd < 0)
			return ER_FOPEN;
	}
	else
		return ER_FOPEN;
    
	write_pair(ini->globals, fd);
	write_section(ini->sections, fd);
    
	close(fd);
    
	/*
	
    FILE *f;

    if(fname) {
        f = fopen(fname, "w");
        if(!f)
            return ER_FOPEN;
    } else
        f = stdout;

    write_pair(ini->globals, f);
    write_section(ini->sections, f);

    if(fname)
        fclose(f);*/

    return SUCCESS;
}

/****************************************************************************/

int ini_has_section(struct ini_file *ini, const char *sec) {
    return find_section(ini->sections, sec) != RT_NULL;
}

/*
 *    Finds a specific parameter-value pair in the configuration file
 */
static ini_pair *find_param(const struct ini_file *ini,
                            const char *sec,
                            const char *par) {
    ini_section *s;
    ini_pair *p;

    if(!ini) return RT_NULL;

    if(sec) {
        s = find_section(ini->sections, sec);
        if(!s) return RT_NULL;
        p = s->fields;
    } else
        p = ini->globals;

    if(!p) return RT_NULL;

    return find_pair(p, par);
}

/*
 *    Retrieves a parameter 'par' from a section 'sec' within the ini_file 'ini'
 *    and returns its value.
 *    If 'sec' is RT_NULL, the global parameters ini 'ini' are searched.
 *    If the value is not found, 'def' is returned.
 *    It returns a string. Functions like atoi(), atof(), strtol() and even
 *    sscanf() can be used to convert it to the relevant type.
 */
const char *ini_get(struct ini_file *ini,
                    const char *sec,
                    const char *par,
                    const char *def) {
    ini_pair *p;

    p = find_param(ini, sec, par);
    if(!p) {
        if(def)
            ini_put(ini, sec, par, def);
        return def;
    }

    return p->value;
}

/*
 *    Sets a parameter 'par' in section 'sec's value to 'val', replacing the
 *    current value if it already exists, or creates the section if it does not
 *    exist
 */
int ini_put(struct ini_file *ini, const char *sec, const char *par, const char *val) {
    ini_section *s;
    ini_pair *p, **pp;

    if(!ini || !val) return 0;

    p = find_param(ini, sec, par);
    if(p) {
        /* Replace the existing value */
        char *t = p->value;
        if(!(p->value = rt_strdup(val))) {
            p->value = t;
            return 0;
        }
				p->valid = 1;
        rt_free(t);
        return 1;
    }

    if(sec) {
        s = find_section(ini->sections, sec);
        if(!s) {
            /* Create a new section */
            if(!(s = (ini_section*)rt_malloc(sizeof *s))) return 0;
            if(!(s->name = rt_strdup(sec))) {
                rt_free(s);
                return 0;
            }

            s->fields = RT_NULL;
            s->left = s->right = RT_NULL;
						s->valid = 1;
            if(ini->sections)
                insert_section(ini->sections, s);
            else
                ini->sections = s;
        }

        pp = &s->fields;
    } else
        pp = &ini->globals;

    if(!(p = (ini_pair*)rt_malloc(sizeof *p)))
        return 0;

    if(!(p->param = rt_strdup(par)) || !(p->value = rt_strdup(val))) {
        rt_free(p);
        return 0;
    }

    p->left = p->right = RT_NULL;

    if(!*pp)
        *pp = p;
    else
        insert_pair(*pp, p);
		p->valid = 1;
    return 1;
}

/*
 *    ini_putf() takes a printf() style format string and uses vsnprintf() to
 *    pass a value to ini_put(). This function is intended for placing
 *    data types that are not strings into the ini_file
 *
 *    The other parameters are the same as those of ini_put().
 */
int ini_putf(struct ini_file *ini,
            const char *sec,
            const char *par,
            const char *fmt,
            ...) {
    char buffer[MAX_LINE];
    va_list arg;

    va_start(arg, fmt);

#ifdef _MSC_VER /* Microsoft Visual C++? */
    /* VC++ messes around with _'s before vsnprintf(): */
#define    vsnprintf _vsnprintf
#endif

#if 1
    vsnprintf(buffer, MAX_LINE, fmt, arg);
#else
    vsprintf(buffer, fmt, arg );
    RT_ASSERT(strlen(buffer) < MAX_LINE);
#endif
    va_end(arg);

    return ini_put(ini, sec, par, buffer);
}
void ini_delete_section(struct ini_file *ini, const char * sec)
{
	ini_section *s = find_section(ini->sections, sec);
	if (!s) return ;
	s->valid = 0;
	return ;		
}
void ini_delete_item(struct ini_file *ini, const char * sec, const char *par)
{
	ini_section *s = find_section(ini->sections, sec);
	ini_pair *p = RT_NULL;
	if (!s) return ;
	p = find_pair(s->fields,par);
	if (!p) return ;
	p->valid = 0;	
	return ;			
}



rt_err_t ini_file_read(char* filename,char* section,char* key, char* val,rt_size_t vLen)
{
	int err, line;
	struct ini_file* inifile = ini_read(filename,&err,&line);
	char* tval;
	rt_size_t sz = 0;
	if(!inifile) return -RT_ENOMEM;
	tval = (char*)ini_get(inifile,section, key, RT_NULL);
	if(!tval)  {	ini_free(inifile);	return -RT_ENOSYS;}
	
	sz = rt_strlen(tval);
	if (sz > (vLen - 1))
		sz = vLen - 1;
	rt_memcpy(val,tval,sz);
	val[sz] = 0;
	ini_free(inifile);	
	return RT_EOK;
}

rt_err_t ini_file_write(char* filename,char* section,char* key, char* val)
{
	int err, line;
	struct ini_file* inifile = ini_read(filename,&err,&line);
	
	if (!inifile)
		inifile = ini_read(RT_NULL,&err,&line);
	if (!inifile)
	{
		rt_kprintf("make ini file failed\n");
		return -RT_ENOMEM;
	}
	if (1 != ini_put(inifile,section,key,val))
	{
		ini_free(inifile);
		rt_kprintf("ini item insert failed\n");
		return -RT_ERROR;
	}
	if (1 != ini_write(inifile, filename))
	{
		ini_free(inifile);
		rt_kprintf("write ini failed\n");
		return -RT_ERROR;
	}
	ini_free(inifile);	
	return RT_EOK;
}

rt_err_t ini_file_delete(char* filename,char* section,char* key)
{
	int err, line;
	struct ini_file* inifile = ini_read(filename,&err,&line);
	if (!inifile)
		return RT_EOK;
	if(key)
		ini_delete_item(inifile,section,key);
	else
		ini_delete_section(inifile,section);
	if (1 != ini_write(inifile, filename))
	{
		ini_free(inifile);
		rt_kprintf("write modified ini file failed\n");
		return -RT_ERROR;
	}
	ini_free(inifile);	
	return RT_EOK;
}

#if defined(RT_USING_FINSH)
#include <finsh.h>

static void ini_modify(uint8_t argc, char **argv)
{
    if (argc != 5) {
        rt_kprintf("ini_modify filename section item value\n");
        return;
    }
    if (RT_EOK != ini_file_write(argv[1], argv[2], argv[3], argv[4])){
        rt_kprintf("write failed\n");
    }
    else
        rt_kprintf("write successful\n");  
    
}
FINSH_FUNCTION_EXPORT(ini_modify, modify ini file item.);

#if defined(FINSH_USING_MSH)
MSH_CMD_EXPORT(ini_modify, modify ini file item.);
#endif /* defined(FINSH_USING_MSH) */
#endif

#endif



















